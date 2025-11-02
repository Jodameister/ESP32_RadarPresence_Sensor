// File: Config.cpp

#include "Config.h"
#include <WiFiManager.h>    // für setupWiFiManager()
#include <Preferences.h>    // für prefs

// Preferences & network clients
Preferences  prefs;
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// Persistent settings
String g_mqttServer = "10.0.0.2";
String g_mqttPort   = "1883";
String g_mqttTopic  = "radar";
String g_radarRxPin = "16";
String g_radarTxPin = "17";
String g_host       = "radar";
String g_otaPass    = "";

// Dynamic parameters
float    g_maxRangeMeters  = 2.1f;
uint32_t g_holdIntervalMs  = 500;

// Timing & pins
unsigned long lastRadarDataTime = 0;
unsigned long lastRadarPub      = 0;
unsigned long lastStatusPub     = 0;
unsigned long lastWiFiCheck     = 0;

const unsigned long RADAR_INTERVAL_MS = 100;
const unsigned long STATUS_INTERVAL   = 10000;
const unsigned long NO_DATA_TIMEOUT   = 3000;
const unsigned long RESTART_TIMEOUT   = 30000;

const int RADAR_BOOT_PIN = 0;

// Radar internals
uint8_t        radarBuf[64];
uint8_t        radarCount = 0;

bool           otaInProgress        = false;
bool           startConfigPortal   = false;
bool           rebootRequested     = false;
bool           serialResetAttempted = false;
unsigned long  serialResetTime      = 0;
uint32_t       wifiReconnectCount   = 0;
uint32_t       radarTimeoutCount    = 0;
uint32_t       radarSerialRestartCount = 0;

const float    ALPHA             = 0.4f;
const float    RANGE_GATE_SIZE   = 0.7f;

const uint8_t  multiTargetCmd[12] = {
  0xFD,0xFC,0xFB,0xFA,
  0x02,0x00,0x90,0x00,
  0x04,0x03,0x02,0x01
};

RadarTarget    smoothed[3];
unsigned long  lastSeenTime[3] = {0,0,0};
unsigned long  lastZeroPub = 0;

//---------------------------------------------------------
// Helper Functions
//---------------------------------------------------------
char* buildMqttTopic(const char* suffix, char* buffer, size_t bufsize) {
  snprintf(buffer, bufsize, "%s/%s", g_mqttTopic.c_str(), suffix);
  return buffer;
}

void nonBlockingDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    mqttClient.loop();
    yield();
  }
}

//---------------------------------------------------------
// WiFiManager setup
//---------------------------------------------------------
void setupWiFiManager() {
  WiFiManager wm;
  wm.setSaveParamsCallback(saveParamCallback);
  wm.addParameter(new WiFiManagerParameter("mqtt_server","MQTT Server", g_mqttServer.c_str(),16));
  wm.addParameter(new WiFiManagerParameter("mqtt_port","MQTT Port",     g_mqttPort.c_str(),6));
  wm.addParameter(new WiFiManagerParameter("mqtt_topic","MQTT Topic",   g_mqttTopic.c_str(),64));
  wm.addParameter(new WiFiManagerParameter("radar_rx","Radar Rx Pin",   g_radarRxPin.c_str(),3));
  wm.addParameter(new WiFiManagerParameter("radar_tx","Radar Tx Pin",   g_radarTxPin.c_str(),3));
  wm.addParameter(new WiFiManagerParameter("host","Hostname",          g_host.c_str(),20));
  wm.addParameter(new WiFiManagerParameter("otaPass","OTA Password",  g_otaPass.c_str(),10));
  wm.setConfigPortalTimeout(60);
  wm.autoConnect("AutoAP","12345678");
}

//---------------------------------------------------------
// Save parameter callback
//---------------------------------------------------------
void saveParamCallback() {
  prefs.begin("myRadar", false);
  prefs.putString("mqtt_server", g_mqttServer);
  prefs.putString("mqtt_port",   g_mqttPort);
  prefs.putString("mqtt_topic",  g_mqttTopic);
  prefs.putString("radar_rx",    g_radarRxPin);
  prefs.putString("radar_tx",    g_radarTxPin);
  prefs.putString("host",        g_host);
  prefs.putString("otaPass",     g_otaPass);
  prefs.end();
}
