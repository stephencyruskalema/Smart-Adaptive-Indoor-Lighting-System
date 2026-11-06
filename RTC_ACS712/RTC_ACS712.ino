#include <Arduino.h>
#include <esp_arduino_version.h>
#include <WiFi.h>
#include <WebServer.h>        // Built-in ESP32 web server
#include <ArduinoJson.h>
#include "network_config.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>           // Requires "RTClib" library by Adafruit
#include <time.h>

// ----------------------------------------------------------------------------
//  1. PIN MAP
// ----------------------------------------------------------------------------
#define NUM_ZONES 2

const uint8_t PIN_PWM[NUM_ZONES]       = {23, 25};  // -> XY-MOS PWM inputs
const uint8_t PIN_LDR[NUM_ZONES]       = {34, 35};  // ADC1, input-only pins
const uint8_t PIN_RADAR_OUT[NUM_ZONES] = {32, 33};  // LD2410 OUT (fallback)
const int8_t  RADAR_RX[NUM_ZONES]      = {16, 26};  // ESP32 RX  <- LD2410 TX
const int8_t  RADAR_TX[NUM_ZONES]      = {17, 27};  // ESP32 TX  -> LD2410 RX
const uint8_t PIN_ACS712               = 36;        // SP pin (GPIO 36) for current sensor
const int8_t  PIN_SDA                  = 21;
const int8_t  PIN_SCL                  = 22;

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
const uint16_t MAX_DETECT_CM       = 5;    // ignore targets farther than this
const bool     CONFIGURE_RADAR     = false; 
const uint8_t  RADAR_MOVE_SENS     = 5;    
const uint8_t  RADAR_STATIC_SENS   = 5;    
const float    SETBACK_DUTY_PCT    = 0.0f;  // light level when zone is empty (0 = off)

// Illuminance feedback
const bool     LDR_INVERT          = false; 
float          TARGET_ADC[NUM_ZONES] = {2000.0f, 2000.0f}; 
const float    DEADBAND_ADC        = 120.0f; 
const float    KI_PCT_PER_COUNT    = 0.010f; 
const float    MAX_STEP_PCT        = 4.0f;   
const float    OCC_START_DUTY_PCT  = 30.0f;  
const float    MIN_OCC_DUTY_PCT    = 0.0f;   
const float    RAMP_DOWN_STEP_PCT  = 3.0f;   

// Timing (non-blocking scheduler)
const uint32_t LDR_SAMPLE_MS    = 20;
const uint32_t RADAR_POLL_MS    = 1;        
const uint32_t CONTROL_MS       = 250;
const uint32_t LCD_MS           = 500;
const uint32_t LOG_MS           = 1000;

// ----------------------------------------------------------------------------
//  3. DATA TYPES
// ----------------------------------------------------------------------------
enum ControlMode { MODE_A_FULL = 'A', MODE_B_ONOFF = 'B', MODE_C_ADAPTIVE = 'C' };

struct RadarParser {
  uint8_t  buf[48];
  uint8_t  idx = 0;
  bool     present = false;
  uint16_t detectCm = 0;
  uint32_t lastFrameMs = 0;

  void feed(uint8_t b) {
    static const uint8_t HDR[4] = {0xF4, 0xF3, 0xF2, 0xF1};
    static const uint8_t FTR[4] = {0xF8, 0xF7, 0xF6, 0xF5};

    if (idx < 4) {                              
      if (b == HDR[idx]) buf[idx++] = b;
      else { idx = 0; if (b == HDR[0]) buf[idx++] = b; }
      return;
    }
    buf[idx++] = b;
    if (idx == 6) {                             
      uint16_t len = buf[4] | (buf[5] << 8);
      if (len < 13 || len > 32) { idx = 0; return; }
    }
    if (idx >= 6) {
      uint16_t len   = buf[4] | (buf[5] << 8);
      uint8_t  total = 6 + len + 4;
      if (idx >= total) {
        bool footerOk = true;
        for (uint8_t i = 0; i < 4; i++)
          if (buf[6 + len + i] != FTR[i]) footerOk = false;
        const uint8_t* d = &buf[6];
        if (footerOk && d[0] == 0x02 && d[1] == 0xAA && d[2] <= 3 && d[11] == 0x55 && d[12] == 0x00) {
          detectCm    = d[9] | (d[10] << 8);
          present     = (d[2] != 0) && (detectCm <= MAX_DETECT_CM);
          lastFrameMs = millis();
        }
        idx = 0;
      }
      if (idx >= sizeof(buf)) idx = 0;          
    }
  }
};

struct Zone {
  RadarParser radar;
  bool     occupied      = false;
  bool     prevOccupied  = false;
  uint32_t lastPresentMs = 0;
  float    ldrFiltered   = 0;     
  float    dutyPct       = 0;     
  bool     radarUartOk   = false;
  bool     enabled       = true;
  bool     manualOverride= false;
  float    manualPct     = 0;
};

// ----------------------------------------------------------------------------
//  4. GLOBALS
// ----------------------------------------------------------------------------
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
RTC_DS3231  rtc;
Zone        zones[NUM_ZONES];
ControlMode mode = MODE_C_ADAPTIVE;

float    systemCurrentAmps = 0.0f;
uint32_t tLdr = 0, tControl = 0, tLcd = 0, tLog = 0;

// ----------------------------------------------------------------------------
//  5. PWM & SENSORS (ACS712 & LDR)
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

void currentSensorUpdate() {
  int rawAdc = analogRead(PIN_ACS712);
  float voltage = (rawAdc / 4095.0f) * 3.3f;
  const float sensitivity = 0.185f; 
  const float zeroVoltage = 2.50f;  
  
  systemCurrentAmps = (voltage - zeroVoltage) / sensitivity;
  if (systemCurrentAmps < 0.05f) {
    systemCurrentAmps = 0.0f;
  }
}

void ldrUpdate() {                              
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    int raw = analogRead(PIN_LDR[z]);
    if (LDR_INVERT) raw = 4095 - raw;
    if (zones[z].ldrFiltered == 0) zones[z].ldrFiltered = raw;   
    zones[z].ldrFiltered += 0.1f * (raw - zones[z].ldrFiltered);
  }
}

void radarDrain() {                             
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    while (radarSerial[z]->available()) zones[z].radar.feed(radarSerial[z]->read());
  }
}

void occupancyUpdate() {
  uint32_t now = millis();
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    Zone& Z = zones[z];
    Z.radarUartOk = (Z.radar.lastFrameMs != 0) && (now - Z.radar.lastFrameMs < RADAR_TIMEOUT_MS);

    bool rawPresent = Z.radarUartOk ? Z.radar.present : (digitalRead(PIN_RADAR_OUT[z]) == HIGH);

    Z.prevOccupied = Z.occupied;
    if (rawPresent) {
      Z.lastPresentMs = now;
      Z.occupied = true;
    } else if (Z.occupied && (now - Z.lastPresentMs > HOLD_TIME_MS)) {
      Z.occupied = false;                       
    }
  }
}

// ----------------------------------------------------------------------------
//  6. CONTROL LOGIC
// ----------------------------------------------------------------------------
void controlZone(uint8_t z) {
  Zone& Z = zones[z];

  if (!Z.enabled) { Z.dutyPct = 0; pwmWritePercent(z, 0); return; }
  if (Z.manualOverride) { Z.dutyPct = Z.manualPct; pwmWritePercent(z, Z.dutyPct); return; }

  switch (mode) {
    case MODE_A_FULL:                           
      Z.dutyPct = 100.0f;
      break;

    case MODE_B_ONOFF:                          
      Z.dutyPct = Z.occupied ? 100.0f : 0.0f;
      break;

    case MODE_C_ADAPTIVE:
    default:
      if (!Z.occupied) {
        if (Z.dutyPct > SETBACK_DUTY_PCT) {
          Z.dutyPct -= RAMP_DOWN_STEP_PCT;
          if (Z.dutyPct < SETBACK_DUTY_PCT) Z.dutyPct = SETBACK_DUTY_PCT;
        } else {
          Z.dutyPct = SETBACK_DUTY_PCT;
        }
      } else {
        if (!Z.prevOccupied && Z.dutyPct < OCC_START_DUTY_PCT) Z.dutyPct = OCC_START_DUTY_PCT;

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
//  7. DISPLAY & LOGGING
// ----------------------------------------------------------------------------
void lcdUpdate() {
  char line[17];
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    int lightPct = (int)(zones[z].ldrFiltered * 100.0f / 4095.0f + 0.5f);
    int dutyPct  = (int)(zones[z].dutyPct + 0.5f);
    snprintf(line, sizeof(line), "Z%d %c L%3d D%3d%%%c",
             z + 1, zones[z].occupied ? 'O' : '-', lightPct, dutyPct,
             (z == 0) ? (char)mode : ' ');
    lcd.setCursor(0, z);
    lcd.print(line);
  }
}

void serialLog() {
  DateTime nowRtc = rtc.now();
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    Serial.printf("%04d-%02d-%02d %02d:%02d:%02d,%lu,%c,%d,%d,%d,%.0f,%.0f,%.1f,%.2fA\n",
                  nowRtc.year(), nowRtc.month(), nowRtc.day(),
                  nowRtc.hour(), nowRtc.minute(), nowRtc.second(),
                  (unsigned long)millis(), (char)mode, z + 1,
                  zones[z].occupied, zones[z].radarUartOk,
                  zones[z].ldrFiltered, TARGET_ADC[z], zones[z].dutyPct,
                  systemCurrentAmps);
  }
}

void serialCommands() {
  while (Serial.available()) {
    char c = toupper(Serial.read());
    if (c == 'A' || c == 'B' || c == 'C') {
      mode = (ControlMode)c;
      Serial.printf("# Mode set to %c\n", c);
    } else if (c == 'T') {                      
      for (uint8_t z = 0; z < NUM_ZONES; z++) TARGET_ADC[z] = zones[z].ldrFiltered;
      Serial.printf("# Targets set to current light\n");
    }
  }
}

// ----------------------------------------------------------------------------
//  8. SETUP / LOOP & SYNCHRONOUS WEB SERVER
// ----------------------------------------------------------------------------
struct NetworkCommand { char mode; int8_t zone; bool hasEnabled, enabled, hasManual, manual; float brightness; bool hasBrightness; };
QueueHandle_t commandQueue;
SemaphoreHandle_t snapshotLock;
WebServer httpServer(HTTP_PORT);
String cachedStatus = "{}";
uint32_t lastWifiAttempt = 0;
const char* bootId = nullptr;
char bootIdBuffer[17];

void publishSnapshot() {
  if(!snapshotLock) return;
  JsonDocument doc;
  DateTime nowRtc = rtc.now();
  doc["apiVersion"] = 1;
  doc["bootId"] = bootId;
  doc["uptimeMs"] = millis();
  char timeStr[20];
  snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d %02d:%02d:%02d", 
           nowRtc.year(), nowRtc.month(), nowRtc.day(), nowRtc.hour(), nowRtc.minute(), nowRtc.second());
  doc["rtcTime"] = timeStr;
  doc["systemCurrentAmps"] = systemCurrentAmps;
  doc["systemStatus"] = "running";
  doc["mode"] = String((char)mode);
  
  JsonArray a = doc["zones"].to<JsonArray>();
  for (uint8_t z=0; z<NUM_ZONES; z++) {
    JsonObject o = a.add<JsonObject>();
    o["id"]=z+1; o["occupied"]=zones[z].occupied;
    o["adc"]=roundf(zones[z].ldrFiltered); o["targetAdc"]=TARGET_ADC[z];
    o["brightness"]=zones[z].dutyPct; o["radarUartOk"]=zones[z].radarUartOk;
    o["enabled"]=zones[z].enabled; o["manualOverride"]=zones[z].manualOverride;
    o["manualBrightness"]=zones[z].manualPct;
    o["controlOwner"]=!zones[z].enabled?"disabled":zones[z].manualOverride?"manual":"automatic";
  }
  String next; serializeJson(doc,next);
  if(xSemaphoreTake(snapshotLock, pdMS_TO_TICKS(5))==pdTRUE) { cachedStatus=next; xSemaphoreGive(snapshotLock); }
}

void jsonError(int code, const char* message) {
  JsonDocument d; d["error"]=message; String out; serializeJson(d,out); 
  httpServer.send(code, "application/json", out);
}

bool authorized() {
  if(!httpServer.hasHeader("Authorization") || httpServer.header("Authorization") != String("Bearer ") + API_TOKEN) {
    jsonError(401, "Invalid API token"); return false;
  }
  return true;
}

void sendSnapshot() {
  if(!authorized()) return;
  String out;
  if(xSemaphoreTake(snapshotLock, pdMS_TO_TICKS(5)) != pdTRUE) { jsonError(503, "Snapshot busy; retry"); return; }
  out = cachedStatus; xSemaphoreGive(snapshotLock);
  httpServer.sendHeader("Cache-Control", "no-store");
  httpServer.send(200, "application/json", out);
}

void handleCommand(int8_t zone) {
  if(!authorized()) return;
  if(!httpServer.hasArg("plain")) { jsonError(400, "Missing JSON body"); return; }
  String raw = httpServer.arg("plain");
  if(raw.length() > 256) { jsonError(413, "Command too large"); return; }
  
  JsonDocument d;
  if(deserializeJson(d, raw) || !d.is<JsonObject>()) { jsonError(400, "Invalid JSON object"); return; }
  
  NetworkCommand cmd{}; cmd.zone = zone;
  if(zone < 0) {
    if(!d["mode"].is<const char*>()) { jsonError(400, "Expected mode A, B or C"); return; }
    String m = d["mode"].as<String>();
    if(m != "A" && m != "B" && m != "C") { jsonError(422, "Mode must be A, B or C"); return; }
    cmd.mode = m[0];
  } else {
    cmd.hasEnabled = d["enabled"].is<bool>(); cmd.enabled = d["enabled"] | false;
    cmd.hasManual = d["manualOverride"].is<bool>(); cmd.manual = d["manualOverride"] | false;
    cmd.hasBrightness = d["brightness"].is<float>() || d["brightness"].is<int>();
    cmd.brightness = d["brightness"] | 0.0f;
    if(cmd.hasBrightness && (!isfinite(cmd.brightness) || cmd.brightness < 0 || cmd.brightness > 100)) { jsonError(422, "Brightness outside 0..100"); return; }
  }
  
  if(xQueueSend(commandQueue, &cmd, 0) != pdTRUE) { jsonError(503, "Command queue full"); return; }
  httpServer.send(202, "application/json", "{\"accepted\":true,\"message\":\"Queued; verify using status\"}");
}

void applyNetworkCommands() {
  if(!commandQueue) return;
  NetworkCommand c;
  if(xQueueReceive(commandQueue, &c, 0) == pdTRUE) {
    if(c.zone < 0) mode = (ControlMode)c.mode;
    else {
      Zone& z = zones[c.zone];
      if(c.hasEnabled) z.enabled = c.enabled;
      if(c.hasBrightness) z.manualPct = c.brightness;
      if(c.hasManual) z.manualOverride = c.manual;
      if(c.hasManual && !c.manual && mode == MODE_C_ADAPTIVE) z.dutyPct = z.occupied ? OCC_START_DUTY_PCT : SETBACK_DUTY_PCT;
    }
    for(uint8_t z = 0; z < NUM_ZONES; z++) {
      if(!zones[z].enabled || zones[z].manualOverride) {
        zones[z].dutyPct = zones[z].enabled ? zones[z].manualPct : 0;
        pwmWritePercent(z, zones[z].dutyPct);
      }
    }
    publishSnapshot();
  }
}

void networkSetup() {
  commandQueue = xQueueCreate(8, sizeof(NetworkCommand));
  snapshotLock = xSemaphoreCreateMutex();
  if(!commandQueue || !snapshotLock) { Serial.println("# Network disabled: allocation failure"); return; }
  
  snprintf(bootIdBuffer, sizeof(bootIdBuffer), "%08lx%08lx", (unsigned long)esp_random(), (unsigned long)esp_random()); 
  bootId = bootIdBuffer;
  publishSnapshot();
  
  if(strlen(API_TOKEN) < 24 || String(API_TOKEN).startsWith("REPLACE_")) { Serial.println("# Network disabled: configure unique API token"); return; }
  
  WiFi.mode(WIFI_STA); 
  WiFi.setAutoReconnect(true); 
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttempt = millis();

  // Wait up to 10 seconds for Wi-Fi connection so we can fetch real-time via NTP
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
    delay(500);
  }

  // Configure NTP time servers (East Africa Time: UTC+3 = 10800 seconds offset)
  configTime(10800, 0, "pool.ntp.org", "time.google.com");

  const char* headerKeys[] = {"Authorization"};
  httpServer.collectHeaders(headerKeys, 1);

  httpServer.on("/api/status", HTTP_GET, sendSnapshot);
  httpServer.on("/api/zones", HTTP_GET, sendSnapshot);
  httpServer.on("/api/health", HTTP_GET, sendSnapshot);
  httpServer.on("/api/history", HTTP_GET, [](){ if(authorized()) httpServer.send(200, "application/json", "{\"storage\":\"phone_sqlite\",\"records\":[]}"); });
  httpServer.on("/api/mode", HTTP_POST, [](){ handleCommand(-1); });
  httpServer.on("/api/zones/1", HTTP_POST, [](){ handleCommand(0); });
  httpServer.on("/api/zones/2", HTTP_POST, [](){ handleCommand(1); });
  httpServer.onNotFound([](){ jsonError(404, "Endpoint not found"); });
  
  httpServer.begin();
}

void networkMaintain() {
  if(!commandQueue || !snapshotLock) return;
  static bool wasOnline = false;
  bool online = WiFi.status() == WL_CONNECTED;
  if(online && !wasOnline) { Serial.print("# App address: http://"); Serial.println(WiFi.localIP()); }
  wasOnline = online;
  if(!online && millis() - lastWifiAttempt >= 30000) { lastWifiAttempt = millis(); WiFi.reconnect(); }
}

void setup() {
  Serial.begin(115200);
  pwmInit();                                    

  analogReadResolution(12);
  
  // Setup LDR and ACS712 pin attenuations
  for (uint8_t z = 0; z < NUM_ZONES; z++) {
    analogSetPinAttenuation(PIN_LDR[z], ADC_11db);   
    pinMode(PIN_RADAR_OUT[z], INPUT);
    radarSerial[z]->begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX[z], RADAR_TX[z]);
  }
  analogSetPinAttenuation(PIN_ACS712, ADC_11db); // Configure SP / GPIO 36 for ADC reading

  // I2C & RTC Init
  Wire.begin(PIN_SDA, PIN_SCL);
  bool rtcAvailable = rtc.begin();
  if (!rtcAvailable) {
    Serial.println("# WARNING: Couldn't find DS3231 RTC module!");
  }
  
  // Setup network and wait briefly for NTP sync
  networkSetup();

  // Automatically sync DS3231 RTC from internet NTP time if connected
  if (rtcAvailable && WiFi.status() == WL_CONNECTED) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 5000)) { // wait up to 5 seconds for internet time
      rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                          timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
      Serial.println("# RTC successfully synced from Internet NTP time!");
    }
  }

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Smart Lighting");
  lcd.setCursor(0, 1); lcd.print("Group C - ESP32");
  delay(1500);                                  
  lcd.clear();

  Serial.println("# timestamp,mode,zone,occupied,radarUART,ldrAdc,targetAdc,duty%,currentA");
}

void loop() {
  uint32_t now = millis();

  httpServer.handleClient(); // Process incoming web requests
  applyNetworkCommands();
  networkMaintain();
  radarDrain();                                 
  serialCommands();

  if (now - tLdr >= LDR_SAMPLE_MS) {
    tLdr = now;
    ldrUpdate();
    currentSensorUpdate(); // Sample current sensor alongside LDRs
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
    publishSnapshot();
  }
}