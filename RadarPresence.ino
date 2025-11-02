//************************************************************
// ESP32 RadarPresence_v1.3e – vollständiger Sketch mit MQTT‑Befehlen
//************************************************************
//
// • WiFiManager + Auto‑Reconnect (Event + Polling)
// • Radar RD‑03D seriell auslesen & exponentiell glätten (Y invertiert)
// • MQTT‑Publish: Targets alle 100 ms, Status alle 10 s
//   – Nur einmal “0 targets” melden, bis neue Targets kommen
// • Hold‑Intervall dynamisch per MQTT einstellbar
// • OTA‑Updates mit Fortschrittsanzeige
// • JSON‑Pufferprüfung, nicht‑blockierende Serial1‑Resets
// • MQTT‑Befehle: config, reboot, resetRadar, setRange:<m>, setHold:<ms>, getStatus
//************************************************************

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <math.h>

// ---------------------------------------------------------
// Firmware‑Version
// ---------------------------------------------------------
#define FW_VERSION "v1.3e"

// ---------------------------------------------------------
// Konstanten
// ---------------------------------------------------------
#define BOOT_PIN               0
#define RADAR_FRAME_SIZE       30
#define RADAR_TARGET_BLOCKSIZE 8
static const float ALPHA    = 0.4f;

const unsigned long STATUS_INTERVAL   = 10000UL; // 10 s
const unsigned long RADAR_INTERVAL_MS = 100UL;   // 0.1 s
const unsigned long NO_DATA_TIMEOUT   = 3000UL;  // 3 s
const unsigned long RESTART_TIMEOUT   = 30000UL; // 30 s

// ---------------------------------------------------------
// Globale Variablen
// ---------------------------------------------------------
Preferences   prefs;
WiFiClient    wifiClient;
PubSubClient  mqttClient(wifiClient);

String g_mqttServer = "10.0.0.2";
String g_mqttPort   = "1883";
String g_mqttTopic  = "radar";
String g_radarRxPin = "16";
String g_radarTxPin = "17";
String g_host       = "radar";
String g_otaPass    = "";

// === Dynamische Parameter ===
static float   g_maxRangeMeters  = 2.1f;    // Reichweite in Metern
static uint32_t g_holdIntervalMs = 500UL;   // Hold‑Intervall in ms

static const float RANGE_GATE_SIZE = 0.7f;  // ein Gate = 0,7 m

bool debugMode = false;

uint8_t  radarBuf[64];
uint8_t  radarCount = 0;

bool     otaInProgress     = false;
bool     startConfigPortal = false;
bool     rebootRequested   = false;

unsigned long lastRadarDataTime = 0;
unsigned long lastRadarPub      = 0;
unsigned long lastStatusPub     = 0;
unsigned long lastWiFiCheck     = 0;

bool          serialResetAttempted = false;
unsigned long serialResetTime      = 0;

uint32_t wifiReconnectCount = 0;
uint32_t radarTimeoutCount  = 0;

struct RadarTarget {
  bool  presence;
  float x, y, speed, distRaw, distanceXY, angleDeg;
};
static RadarTarget smoothed[3];
static unsigned long lastSeenTime[3] = {0,0,0};
static unsigned long lastZeroPub = 0;

// Multi‑Target‑CMD
const uint8_t multiTargetCmd[12] = {
  0xFD,0xFC,0xFB,0xFA,
  0x02,0x00,0x90,0x00,
  0x04,0x03,0x02,0x01
};

// WiFiManager‑Parameter
struct ParamInfo {
  const char* id; const char* label; String& val; int len; const char* html;
};
ParamInfo paramInfos[] = {
  {"mqtt_server","MQTT Server", g_mqttServer, 16, "type='text' maxlength='15'"},
  {"mqtt_port",  "MQTT Port",   g_mqttPort,   6,  "type='text' maxlength='5'"},
  {"mqtt_topic", "MQTT Topic",  g_mqttTopic,  64, "type='text' maxlength='63'"},
  {"radar_rx",   "Radar Rx Pin",g_radarRxPin, 3,  "type='text' maxlength='2'"},
  {"radar_tx",   "Radar Tx Pin",g_radarTxPin, 3,  "type='text' maxlength='2'"},
  {"host",       "Hostname",    g_host,       20, "type='text' maxlength='20'"},
  {"otaPass",    "OTA Password",g_otaPass,    10, "type='text' maxlength='10'"}
};
WiFiManagerParameter paramObjects[sizeof(paramInfos)/sizeof(paramInfos[0])];

// ---------------------------------------------------------
// Forward‑Deklarationen
// ---------------------------------------------------------
void configureWiFiManager(WiFiManager& wm);
void setupWiFiManagerParams(WiFiManager& wm);
void saveParamCallback();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttReconnect();
void maintainMqtt();
void handleMqttCommands();
void otaSetup();
bool isBlockEmpty(const uint8_t* b);
int16_t parseSigned16(uint8_t lo, uint8_t hi);
uint16_t parseUnsigned16(uint8_t lo, uint8_t hi);
void parseRadarFrame(const uint8_t* buf, uint8_t len);
void readRadarData();
void checkRadarConnection();
void buildAndPublishJson();
void publishStatus();

// Neue Funktionen für MQTT‑Befehle
void restartRadarSerial();
void setMaxRadarRange(float maxMeters);
void setHoldInterval(uint32_t holdMs);

// ---------------------------------------------------------
// Helper: Max‑Range per UART ins Gate übersetzen
// ---------------------------------------------------------
void setMaxRadarRange(float maxMeters) {
  g_maxRangeMeters = maxMeters;
  uint8_t gate = (uint8_t)ceil(maxMeters / RANGE_GATE_SIZE);
  if (gate > 15) gate = 15;

  const uint8_t openCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x04,0x00, 0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(openCmd, sizeof(openCmd));
  delay(50);

  const uint8_t setCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x08,0x00, 0x07,0x00, 0x01,0x00,
    gate,0x00,0x00,0x00, 0x04,0x03,0x02,0x01
  };
  Serial1.write(setCmd, sizeof(setCmd));
  delay(50);

  const uint8_t closeCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x02,0x00, 0xFE,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(closeCmd, sizeof(closeCmd));
  delay(50);

  char buf[64];
  snprintf(buf, sizeof(buf), "maxRange set to %.2f m", g_maxRangeMeters);
  mqttClient.publish((g_mqttTopic + "/ack").c_str(), buf);
}

// ---------------------------------------------------------
// Helper: Hold‑Interval per MQTT setzen
// ---------------------------------------------------------
void setHoldInterval(uint32_t holdMs) {
  g_holdIntervalMs = holdMs;
  char buf[64];
  snprintf(buf, sizeof(buf), "holdInterval set to %u ms", g_holdIntervalMs);
  mqttClient.publish((g_mqttTopic + "/ack").c_str(), buf);
}

// ---------------------------------------------------------
// MQTT‑gesteuerter Serial‑Reset
// ---------------------------------------------------------
void restartRadarSerial() {
  Serial.println("MQTT 'resetRadar' → Serial1 neu starten");
  Serial1.end();
  delay(50);
  Serial1.begin(256000, SERIAL_8N1,
                g_radarRxPin.toInt(),
                g_radarTxPin.toInt());
  delay(50);
  setMaxRadarRange(g_maxRangeMeters);
  Serial1.write(multiTargetCmd, sizeof(multiTargetCmd));
  mqttClient.publish((g_mqttTopic + "/ack").c_str(), "resetRadar OK");
}

// ---------------------------------------------------------
// WiFiManager‑Helper
// ---------------------------------------------------------
void configureWiFiManager(WiFiManager& wm) {
  wm.setSaveParamsCallback(saveParamCallback);
  setupWiFiManagerParams(wm);
  const char* menuItems[] = {"wifi","param","info","restart","exit","update"};
  wm.setMenu(menuItems, sizeof(menuItems)/sizeof(menuItems[0]));
  wm.setConfigPortalTimeout(60);
}
void setupWiFiManagerParams(WiFiManager& wm) {
  for (size_t i=0; i<sizeof(paramInfos)/sizeof(paramInfos[0]); i++) {
    new(&paramObjects[i]) WiFiManagerParameter(
      paramInfos[i].id,
      paramInfos[i].label,
      paramInfos[i].val.c_str(),
      paramInfos[i].len,
      paramInfos[i].html
    );
    wm.addParameter(&paramObjects[i]);
  }
}
void saveParamCallback() {
  prefs.begin("myRadar", false);
  for (size_t i=0; i<sizeof(paramInfos)/sizeof(paramInfos[0]); i++) {
    prefs.putString(paramInfos[i].id, paramObjects[i].getValue());
  }
  prefs.end();
}

// ---------------------------------------------------------
// Radar‑Helper
// ---------------------------------------------------------
bool isBlockEmpty(const uint8_t* b) {
  for (int i=0; i<RADAR_TARGET_BLOCKSIZE; i++)
    if (b[i]!=0) return false;
  return true;
}
int16_t parseSigned16(uint8_t lo, uint8_t hi) {
  bool pos = (hi & 0x80) != 0;
  int16_t v = ((hi & 0x7F) << 8) | lo;
  return pos ? v : -v;
}
uint16_t parseUnsigned16(uint8_t lo, uint8_t hi) {
  return ((uint16_t)hi << 8) | lo;
}

// ---------------------------------------------------------
// setup()
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(BOOT_PIN, INPUT_PULLUP);

  prefs.begin("myRadar", true);
  g_mqttServer = prefs.getString("mqtt_server", g_mqttServer);
  g_mqttPort   = prefs.getString("mqtt_port",   g_mqttPort);
  g_mqttTopic  = prefs.getString("mqtt_topic",  g_mqttTopic);
  g_radarRxPin = prefs.getString("radar_rx",    g_radarRxPin);
  g_radarTxPin = prefs.getString("radar_tx",    g_radarTxPin);
  g_host       = prefs.getString("host",        g_host);
  g_otaPass    = prefs.getString("otaPass",     g_otaPass);
  prefs.end();

  WiFiManager wm;
  configureWiFiManager(wm);
  bool ok = wm.autoConnect("AutoConnectAP","12345678");
  Serial.println(ok ? "WiFi verbunden" : "WiFiManager Timeout");
  if (ok) WiFi.setTxPower(WIFI_POWER_19_5dBm);

  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.onEvent(
    [](arduino_event_id_t, WiFiEventInfo_t){
      Serial.println("WLAN getrennt – reconnect…");
      wifiReconnectCount++;
      WiFi.reconnect();
    },
    ARDUINO_EVENT_WIFI_STA_DISCONNECTED
  );

  otaSetup();
  WiFi.setHostname(g_host.c_str());

  Serial1.begin(256000, SERIAL_8N1,
                g_radarRxPin.toInt(),
                g_radarTxPin.toInt());
  delay(100);

  setMaxRadarRange(g_maxRangeMeters);
  Serial1.write(multiTargetCmd, sizeof(multiTargetCmd));

  mqttClient.setServer(g_mqttServer.c_str(), g_mqttPort.toInt());
  mqttClient.setCallback(mqttCallback);
  mqttReconnect();

  lastRadarDataTime = millis();
}

// ---------------------------------------------------------
// loop()
// ---------------------------------------------------------
void loop() {
  if (WiFi.status() != WL_CONNECTED &&
      millis() - lastWiFiCheck > 10000UL) {
    Serial.println("WLAN nicht verbunden – reconnect…");
    wifiReconnectCount++;
    WiFi.reconnect();
    lastWiFiCheck = millis();
  }

  ArduinoOTA.handle();
  if (otaInProgress) return;

  maintainMqtt();
  readRadarData();

  unsigned long now = millis();
  if (now - lastRadarPub >= RADAR_INTERVAL_MS) {
    lastRadarPub = now;
    buildAndPublishJson();
  }
  if (now - lastStatusPub >= STATUS_INTERVAL) {
    lastStatusPub = now;
    publishStatus();
  }

  handleMqttCommands();
  checkRadarConnection();

  delay(1);
}

// ---------------------------------------------------------
// mqttCallback()
// ---------------------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i=0; i<length; i++) msg += (char)payload[i];
  msg.trim();

  if (msg == "config") {
    mqttClient.publish((g_mqttTopic + "/ack").c_str(), "config OK");
    startConfigPortal = true;
  }
  else if (msg == "reboot") {
    mqttClient.publish((g_mqttTopic + "/ack").c_str(), "reboot OK");
    rebootRequested = true;
  }
  else if (msg == "resetRadar") {
    restartRadarSerial();
  }
  else if (msg.startsWith("setRange:")) {
    float val = msg.substring(9).toFloat();
    setMaxRadarRange(val);
  }
  else if (msg.startsWith("setHold:")) {
    uint32_t h = msg.substring(8).toInt();
    setHoldInterval(h);
  }
  else if (msg == "getStatus") {
    publishStatus();
    mqttClient.publish((g_mqttTopic + "/ack").c_str(), "getStatus OK");
  }
  else if (msg == "help") {
    const char* cmds =
      "Verfügbare Kommandos:\n"
      "• config\n"
      "• reboot\n"
      "• resetRadar\n"
      "• setRange:<m>\n"
      "• setHold:<ms>\n"
      "• getStatus\n"
      "• help";
    mqttClient.publish((g_mqttTopic + "/ack").c_str(), cmds);
  }
  else if (msg == "debug") {
    debugMode = !debugMode;
    mqttClient.publish((g_mqttTopic + "/ack").c_str(),
      debugMode ? "DEBUG ON" : "DEBUG OFF");
  }
}

// ---------------------------------------------------------
// mqttReconnect()
// ---------------------------------------------------------
void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Versuche MQTT zu verbinden... ");
    String id = "RD03D-" + String(random(0xffff), HEX);
    int rc = mqttClient.connect(id.c_str());
    if (rc == 1) {
      Serial.println("verbunden");
      mqttClient.subscribe((g_mqttTopic + "/cmd").c_str());
    } else {
      Serial.print("fehlgeschlagen, rc="); Serial.println(rc);
      delay(2000);
    }
  }
}

// ---------------------------------------------------------
// maintainMqtt()
// ---------------------------------------------------------
void maintainMqtt() {
  if (!mqttClient.connected()) mqttReconnect();
  mqttClient.loop();
}

// ---------------------------------------------------------
// handleMqttCommands()
// ---------------------------------------------------------
void handleMqttCommands() {
  if (startConfigPortal) {
    startConfigPortal = false;
    Serial.println("MQTT 'config' → öffne Config‑Portal");
    WiFiManager wm2;
    configureWiFiManager(wm2);
    wm2.startConfigPortal("OnDemandAP","12345678");
    Serial.println("Portal beendet → reconnect WIFI");
    WiFi.reconnect();
  }
  if (rebootRequested) {
    rebootRequested = false;
    ESP.restart();
  }
}

// ---------------------------------------------------------
// parseRadarFrame()
// ---------------------------------------------------------
void parseRadarFrame(const uint8_t* buf, uint8_t len) {
  if (len != RADAR_FRAME_SIZE) return;
  if (buf[0]!=0xAA || buf[1]!=0xFF || buf[2]!=0x03 || buf[3]!=0x00) return;

  unsigned long now = millis();
  for (int i=0; i<3; i++) {
    const uint8_t* blk = buf + 4 + i*RADAR_TARGET_BLOCKSIZE;
    RadarTarget cur;
    bool seen = !isBlockEmpty(blk);

    if (seen) {
      lastSeenTime[i] = now;
      int16_t rawX = parseSigned16(blk[0],blk[1]);
      int16_t rawY = parseSigned16(blk[2],blk[3]);
      cur.presence   = true;
      cur.x          = rawX;
      cur.y          = rawY;
      cur.speed      = parseSigned16(blk[4],blk[5]);
      cur.distRaw    = parseUnsigned16(blk[6],blk[7]);
      cur.distanceXY = sqrtf(rawX*rawX + rawY*rawY);
      cur.angleDeg   = atan2f(rawY,rawX)*180.0f/PI;
    }
    else if (now - lastSeenTime[i] <= g_holdIntervalMs) {
      cur = smoothed[i];
      cur.presence = true;
    }
    else {
      cur.presence = false;
      cur.x = cur.y = cur.speed = cur.distRaw = cur.distanceXY = cur.angleDeg = 0;
    }

    if (!smoothed[i].presence) {
      smoothed[i] = cur;
    } else if (cur.presence) {
      smoothed[i].x         = ALPHA*smoothed[i].x + (1-ALPHA)*cur.x;
      smoothed[i].y         = ALPHA*smoothed[i].y + (1-ALPHA)*cur.y;
      smoothed[i].speed     = ALPHA*smoothed[i].speed + (1-ALPHA)*cur.speed;
      smoothed[i].distRaw   = cur.distRaw;
      smoothed[i].distanceXY= sqrtf(smoothed[i].x*smoothed[i].x +
                                   smoothed[i].y*smoothed[i].y);
      smoothed[i].angleDeg  = atan2f(smoothed[i].y,smoothed[i].x)*180.0f/PI;
    }
    smoothed[i].presence = cur.presence;
  }
}

// ---------------------------------------------------------
// readRadarData()
// ---------------------------------------------------------
void readRadarData() {
  const uint16_t MAX_READ = 256;
  uint16_t readCnt = 0;
  static uint8_t lastF[RADAR_FRAME_SIZE];

  while (Serial1.available() && readCnt < MAX_READ) {
    lastRadarDataTime = millis();
    if (radarCount >= sizeof(radarBuf)) radarCount = 0;
    radarBuf[radarCount++] = Serial1.read();
    readCnt++;

    // Frame‑Ende erkannt?
    if (radarCount >= 2 &&
        radarBuf[radarCount-2] == 0x55 &&
        radarBuf[radarCount-1] == 0xCC) {

      if (radarCount == RADAR_FRAME_SIZE) {
        // 1) Debug immer senden:
        if (debugMode) {
          char dbg[3*RADAR_FRAME_SIZE + 1] = {0};
          for (int i = 0; i < RADAR_FRAME_SIZE; i++)
            sprintf(dbg + strlen(dbg), "%02X ", radarBuf[i]);
          mqttClient.publish((g_mqttTopic + "/debug").c_str(), dbg);
        }

        // 2) Prüfen, ob das Frame echte Targets enthält:
        bool hasAnyTarget = false;
        for (int i = 0; i < 3; i++) {
          const uint8_t* blk = radarBuf + 4 + i*RADAR_TARGET_BLOCKSIZE;
          if (!isBlockEmpty(blk)) {
            hasAnyTarget = true;
            break;
          }
        }

        // 3) Parsing auslösen, wenn
        //    a) es neue Target‑Daten sind (memcmp ≠ 0) oder
        //    b) gar keine Targets mehr da sind (!hasAnyTarget)
        if (memcmp(radarBuf, lastF, RADAR_FRAME_SIZE) != 0
            || !hasAnyTarget) {
          memcpy(lastF, radarBuf, RADAR_FRAME_SIZE);
          parseRadarFrame(radarBuf, radarCount);
        }
      }

      radarCount = 0;
    }
  }

  if (Serial1.available()) yield();
}

// ---------------------------------------------------------
// checkRadarConnection()
// ---------------------------------------------------------
void checkRadarConnection() {
  unsigned long now = millis();
  if (now - lastRadarDataTime > NO_DATA_TIMEOUT) {
    if (!serialResetAttempted) {
      Serial.println("Keine Radar‑Daten → Serial1 neu starten");
      restartRadarSerial();
      serialResetAttempted = true;
      serialResetTime      = now;
      radarTimeoutCount++;
    }
    else if (now - serialResetTime > RESTART_TIMEOUT) {
      Serial.println("Serial1 Reset fehlgeschlagen → reboot");
      ESP.restart();
    }
  } else {
    serialResetAttempted = false;
  }
}

// ---------------------------------------------------------
// buildAndPublishJson()
// ---------------------------------------------------------
void buildAndPublishJson() {
  StaticJsonDocument<512> doc;
  int cnt = 0;
  for (auto &t: smoothed) if (t.presence) cnt++;
  doc["targetCount"] = cnt;
  for (int i = 0; i < 3; i++) {
    char key[12];
    snprintf(key, sizeof(key), "target%d", i + 1);
    auto o = doc.createNestedObject(key);
    if (!smoothed[i].presence) {
      o["presence"] = false;
    } else {
      o["presence"]  = true;
      o["x"]         = round(smoothed[i].x);
      o["y"]         = round(smoothed[i].y);
      o["speed"]     = round(smoothed[i].speed);
      o["distRaw"]   = round(smoothed[i].distRaw);
      o["distance"]  = round(smoothed[i].distanceXY);
      o["angleDeg"]  = round(smoothed[i].angleDeg);
    }
  }
  char buf[512];
  serializeJson(doc, buf);
  unsigned long now = millis();

  if (!cnt) {
    // 0 Targets: max. 1× pro Sekunde
    if (now - lastZeroPub < 1000) return;
    lastZeroPub = now;
  }

  Serial.print("Publish radar: "); Serial.println(buf);
  mqttClient.publish(g_mqttTopic.c_str(), buf);
}

// ---------------------------------------------------------
// publishStatus()
// ---------------------------------------------------------
void publishStatus() {
  StaticJsonDocument<512> doc;
  doc["fwVersion"]      = FW_VERSION;
  doc["uptime_min"]     = millis()/60000;
  doc["resetReason"]    = esp_reset_reason();
  doc["rssi"]           = WiFi.RSSI();
  doc["channel"]        = WiFi.channel();
  doc["heap_free"]      = ESP.getFreeHeap();
#ifdef CONFIG_FREERTOS_USE_TRACE_FACILITY
  doc["psram_free"]     = ESP.getFreePsram();
#endif
  doc["temp_c"]         = temperatureRead();
  doc["mqttState"]      = mqttClient.state();
  doc["wifiReconnects"] = wifiReconnectCount;
  doc["radarTimeouts"]  = radarTimeoutCount;
  doc["lastRadarDelta"] = millis() - lastRadarDataTime;
  char buf[512];
  serializeJson(doc, buf);
  Serial.print("Publish status: "); Serial.println(buf);
  mqttClient.publish((g_mqttTopic+"/status").c_str(), buf);
}

// ---------------------------------------------------------
// otaSetup()
// ---------------------------------------------------------
void otaSetup() {
  ArduinoOTA.onStart(   [](){ otaInProgress=true; });
  ArduinoOTA.onEnd(     [](){ otaInProgress=false;});
  ArduinoOTA.onProgress([](unsigned int p,unsigned int t){
    Serial.printf("OTA %u%%\n",(p*100)/t);
  });
  ArduinoOTA.setHostname( g_host.c_str() );
  ArduinoOTA.setPassword( g_otaPass.c_str() );
  ArduinoOTA.begin();
}