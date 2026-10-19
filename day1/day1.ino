
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ----------------------------------------------------------------------------
//  1. PIN MAP
// ----------------------------------------------------------------------------
#define NUM_ZONES 2

const uint8_t PIN_PWM[NUM_ZONES]       = {23, 25};  // -> XY-MOS PWM inputs
const uint8_t PIN_LDR[NUM_ZONES]       = {34, 35};  // ADC1, input-only pins
const uint8_t PIN_RADAR_OUT[NUM_ZONES] = {32, 33};  // LD2410 OUT (fallback)
const int8_t  RADAR_RX[NUM_ZONES]      = {16, 26};  // ESP32 RX  <- LD2410 TX
const int8_t  RADAR_TX[NUM_ZONES]      = {17, 27};  // ESP32 TX  -> LD2410 RX
const int8_t  PIN_SDA = 21;
const int8_t  PIN_SCL = 22;

HardwareSerial* radarSerial[NUM_ZONES] = {&Serial2, &Serial1};

// ----------------------------------------------------------------------------
//  2. TUNABLE SETTINGS  (calibrate these on your bench!)
// ----------------------------------------------------------------------------
const uint8_t  LCD_ADDR        = 0x27;      // try 0x3F if the screen stays blank
const uint32_t RADAR_BAUD      = 256000;    // LD2410 factory default

// PWM
const uint32_t PWM_FREQ_HZ     = 5000;
const uint8_t  PWM_BITS        = 10;        // 0..1023
const uint16_t PWM_MAX         = (1 << PWM_BITS) - 1;

// Occupancy
const uint32_t HOLD_TIME_MS        = 15000; // keep "occupied" this long after last detection
const uint32_t RADAR_TIMEOUT_MS    = 2000;  // no UART frame for this long -> use OUT pin
const float    SETBACK_DUTY_PCT    = 0.0f;  // light level when zone is empty (0 = off)

// Illuminance feedback (raw ADC counts 0..4095; HIGHER = BRIGHTER by default)
const bool     LDR_INVERT          = false; // set true if your divider is wired the other way
float          TARGET_ADC[NUM_ZONES] = {2000.0f, 2000.0f}; // desired work-plane level
const float    DEADBAND_ADC        = 120.0f; // +/- band in which duty is not changed
const float    KI_PCT_PER_COUNT    = 0.010f; // integral gain per control step
const float    MAX_STEP_PCT        = 4.0f;   // max duty change per control step (anti-flicker)
const float    OCC_START_DUTY_PCT  = 30.0f;  // starting duty when someone enters
const float    MIN_OCC_DUTY_PCT    = 0.0f;   // floor while occupied (set e.g. 10 to never go fully off)
const float    RAMP_DOWN_STEP_PCT  = 3.0f;   // fade-out speed when zone empties

// Timing (non-blocking scheduler)
const uint32_t LDR_SAMPLE_MS    = 20;
const uint32_t RADAR_POLL_MS    = 1;        // drain UART frequently
const uint32_t CONTROL_MS       = 250;
const uint32_t LCD_MS           = 500;
const uint32_t LOG_MS           = 1000;

// ----------------------------------------------------------------------------
//  3. DATA TYPES
// ----------------------------------------------------------------------------
enum ControlMode { MODE_A_FULL = 'A', MODE_B_ONOFF = 'B', MODE_C_ADAPTIVE = 'C' };

// --- Minimal LD2410 basic-mode data-frame parser (no external library) -------
// Frame: F4 F3 F2 F1 | len(2,LE) | data[len] | F8 F7 F6 F5
// data : 02 AA | state | moveDist(2) | moveEn | statDist(2) | statEn | detDist(2) | 55 00
// state: 0 none, 1 moving, 2 stationary, 3 moving+stationary
struct RadarParser {
  uint8_t  buf[48];
  uint8_t  idx = 0;
  bool     present = false;
  uint16_t detectCm = 0;
  uint32_t lastFrameMs = 0;

  void feed(uint8_t b) {
    static const uint8_t HDR[4] = {0xF4, 0xF3, 0xF2, 0xF1};
    static const uint8_t FTR[4] = {0xF8, 0xF7, 0xF6, 0xF5};

    if (idx < 4) {                              // looking for header
      if (b == HDR[idx]) buf[idx++] = b;
      else { idx = 0; if (b == HDR[0]) buf[idx++] = b; }
      return;
    }
    buf[idx++] = b;
    if (idx == 6) {                             // length known
      uint16_t len = buf[4] | (buf[5] << 8);
      if (len < 4 || len > 32) { idx = 0; return; }
    }
    if (idx >= 6) {
      uint16_t len   = buf[4] | (buf[5] << 8);
      uint8_t  total = 6 + len + 4;
      if (idx >= total) {
        bool footerOk = true;
        for (uint8_t i = 0; i < 4; i++)
          if (buf[6 + len + i] != FTR[i]) footerOk = false;
        const uint8_t* d = &buf[6];
        if (footerOk && d[0] == 0x02 && d[1] == 0xAA && d[2] <= 3) {
          present     = (d[2] != 0);
          detectCm    = d[9] | (d[10] << 8);
          lastFrameMs = millis();
        }
        idx = 0;
      }
      if (idx >= sizeof(buf)) idx = 0;          // safety
    }
  }
};

struct Zone {
  RadarParser radar;
  bool     occupied      = false;
  bool     prevOccupied  = false;
  uint32_t lastPresentMs = 0;
  float    ldrFiltered   = 0;     // filtered ADC counts (brightness-oriented)
  float    dutyPct       = 0;     // 0..100
  bool     radarUartOk   = false;
};

// ----------------------------------------------------------------------------
//  4. GLOBALS
// ----------------------------------------------------------------------------
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
Zone        zones[NUM_ZONES];
ControlMode mode = MODE_C_ADAPTIVE;

uint32_t tLdr = 0, tControl = 0, tLcd = 0, tLog = 0;

// ----------------------------------------------------------------------------
//  5. PWM (compatible with ESP32 core 2.x and 3.x)
// ----------------------------------------------------------------------------
void pwmInit() {
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(PIN_PWM[z], PWM_FREQ_HZ, PWM_BITS);
    ledcWrite(PIN_PWM[z], 0);
#else
    ledcSetup(z, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(PIN_PWM[z], z);
    ledcWrite(z, 0);
#endif
  }
}

void pwmWritePercent(uint8_t z, float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  uint32_t duty = (uint32_t)(pct * PWM_MAX / 100.0f + 0.5f);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_PWM[z], duty);
#else
  ledcWrite(z, duty);
#endif
}

// ----------------------------------------------------------------------------
//  6. SENSOR HANDLING
// ----------------------------------------------------------------------------
void ldrUpdate() {                              // exponential moving average
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    int raw = analogRead(PIN_LDR[z]);
    if (LDR_INVERT) raw = 4095 - raw;
    if (zones[z].ldrFiltered == 0) zones[z].ldrFiltered = raw;   // first sample
    zones[z].ldrFiltered += 0.1f * (raw - zones[z].ldrFiltered);
  }
}

void radarDrain() {                             // non-blocking UART read
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    while (radarSerial[z]->available()) zones[z].radar.feed(radarSerial[z]->read());
  }
}

void occupancyUpdate() {
  uint32_t now = millis();
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    Zone& Z = zones[z];
    Z.radarUartOk = (Z.radar.lastFrameMs != 0) && (now - Z.radar.lastFrameMs < RADAR_TIMEOUT_MS);

    // Primary: UART data frames.  Fallback: LD2410 OUT pin (HIGH = presence).
    bool rawPresent = Z.radarUartOk ? Z.radar.present : (digitalRead(PIN_RADAR_OUT[z]) == HIGH);

    Z.prevOccupied = Z.occupied;
    if (rawPresent) {
      Z.lastPresentMs = now;
      Z.occupied = true;
    } else if (Z.occupied && (now - Z.lastPresentMs > HOLD_TIME_MS)) {
      Z.occupied = false;                       // hold time expired
    }
  }
}

// ----------------------------------------------------------------------------
//  7. CONTROL LOGIC
// ----------------------------------------------------------------------------
void controlZone(uint8_t z) {
  Zone& Z = zones[z];

  switch (mode) {
    case MODE_A_FULL:                           // fixed full output
      Z.dutyPct = 100.0f;
      break;

    case MODE_B_ONOFF:                          // occupancy ON/OFF
      Z.dutyPct = Z.occupied ? 100.0f : 0.0f;
      break;

    case MODE_C_ADAPTIVE:
    default:
      if (!Z.occupied) {
        // Empty: fade smoothly down to the setback level
        if (Z.dutyPct > SETBACK_DUTY_PCT) {
          Z.dutyPct -= RAMP_DOWN_STEP_PCT;
          if (Z.dutyPct < SETBACK_DUTY_PCT) Z.dutyPct = SETBACK_DUTY_PCT;
        } else {
          Z.dutyPct = SETBACK_DUTY_PCT;
        }
      } else {
        // Someone just arrived: jump to a sensible start so they are not in the dark
        if (!Z.prevOccupied && Z.dutyPct < OCC_START_DUTY_PCT) Z.dutyPct = OCC_START_DUTY_PCT;

        // Integral feedback: error > 0 means too dark -> brighten
        float err = TARGET_ADC[z] - Z.ldrFiltered;
        if (fabsf(err) > DEADBAND_ADC) {
          float step = constrain(KI_PCT_PER_COUNT * err, -MAX_STEP_PCT, MAX_STEP_PCT);
          Z.dutyPct += step;
        }
        Z.dutyPct = constrain(Z.dutyPct, MIN_OCC_DUTY_PCT, 100.0f);
      }
      break;
  }
  pwmWritePercent(z, Z.dutyPct);
}

// ----------------------------------------------------------------------------
//  8. DISPLAY & LOGGING
// ----------------------------------------------------------------------------
void lcdUpdate() {
  char line[17];
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    int lightPct = (int)(zones[z].ldrFiltered * 100.0f / 4095.0f + 0.5f);
    int dutyPct  = (int)(zones[z].dutyPct + 0.5f);
    // e.g. "Z1 O L 72 D100% " (O = occupied, - = empty); last char of row 1 = mode
    snprintf(line, sizeof(line), "Z%d %c L%3d D%3d%%%c",
             z + 1, zones[z].occupied ? 'O' : '-', lightPct, dutyPct,
             (z == 0) ? (char)mode : ' ');
    lcd.setCursor(0, z);
    lcd.print(line);
  }
}

void serialLog() {
  // CSV: ms,mode,zone,occupied,radarUART,ldrAdc,target,duty%
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    Serial.printf("%lu,%c,%d,%d,%d,%.0f,%.0f,%.1f\n",
                  (unsigned long)millis(), (char)mode, z + 1,
                  zones[z].occupied, zones[z].radarUartOk,
                  zones[z].ldrFiltered, TARGET_ADC[z], zones[z].dutyPct);
  }
}

void serialCommands() {
  while (Serial.available()) {
    char c = toupper(Serial.read());
    if (c == 'A' || c == 'B' || c == 'C') {
      mode = (ControlMode)c;
      Serial.printf("# Mode set to %c\n", c);
    } else if (c == 'T') {                      // quick calibration: current light = target
      for (uint8_t z = 0; z < NUM_ZONES; z++) TARGET_ADC[z] = zones[z].ldrFiltered;
      Serial.printf("# Targets set to current light: Z1=%.0f Z2=%.0f\n", TARGET_ADC[0], TARGET_ADC[1]);
    }
  }
}

// ----------------------------------------------------------------------------
//  9. SETUP / LOOP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pwmInit();                                    // LEDs off at start

  analogReadResolution(12);
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    analogSetPinAttenuation(PIN_LDR[z], ADC_11db);   // 0..~3.3 V range
    pinMode(PIN_RADAR_OUT[z], INPUT);
    radarSerial[z]->begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX[z], RADAR_TX[z]);
  }

  Wire.begin(PIN_SDA, PIN_SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Smart Lighting");
  lcd.setCursor(0, 1); lcd.print("Group C - ESP32");
  delay(1500);                                  // one-time splash only
  lcd.clear();

  Serial.println("# ms,mode,zone,occupied,radarUART,ldrAdc,targetAdc,duty%");
  Serial.println("# Commands: A/B/C = mode, T = set current light as target");
}

void loop() {
  uint32_t now = millis();

  radarDrain();                                 // keep UART buffers empty
  serialCommands();

  if (now - tLdr >= LDR_SAMPLE_MS) {
    tLdr = now;
    ldrUpdate();
  }

  if (now - tControl >= CONTROL_MS) {
    tControl = now;
    occupancyUpdate();
    for (uint8_t z = 0; z < NUM_ZONES; z++) controlZone(z);
  }

  if (now - tLcd >= LCD_MS) {
    tLcd = now;
    lcdUpdate();
  }

  if (now - tLog >= LOG_MS) {
    tLog = now;
    serialLog();
  }
}
