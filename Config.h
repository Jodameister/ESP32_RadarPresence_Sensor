// File: Config.h

#pragma once

#include <Preferences.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Version
#define FW_VERSION "v1.8"

// Constants
#define MQTT_TOPIC_BUFFER_SIZE 80
#define JSON_BUFFER_SIZE 1536
#define RADAR_CMD_DELAY_US 50000  // 50ms in microseconds
#define SERIAL_LOG_LINES 10
#define SERIAL_LOG_LINE_LEN 96

// Preferences & network clients
extern Preferences  prefs;
extern WiFiClient   wifiClient;
extern PubSubClient mqttClient;

// Persistent settings
extern String g_mqttServer, g_mqttPort, g_mqttTopic;
extern String g_radarRxPin, g_radarTxPin;
extern String g_host, g_otaPass;

// Dynamic parameters
extern float   g_maxRangeMeters;
extern uint32_t g_holdIntervalMs;

// Timing & pins
extern unsigned long lastRadarDataTime, lastRadarPub, lastStatusPub, lastWiFiCheck;
extern unsigned long lastWiFiConnected, lastWiFiReconnectAttempt;
extern const unsigned long RADAR_INTERVAL_MS, STATUS_INTERVAL, NO_DATA_TIMEOUT, RESTART_TIMEOUT;
extern const int          RADAR_BOOT_PIN;

// Forward declarations
void setupWiFiManager();
void saveParamCallback();

// Helper functions
char* buildMqttTopic(const char* suffix, char* buffer, size_t bufsize);
void nonBlockingDelay(unsigned long ms);
void formatUptime(char* buffer, size_t bufsize);
void logPrint(const char* msg);
void logPrintln(const char* msg);
void logPrint(const String& msg);
void logPrintln(const String& msg);
void logPrintf(const char* fmt, ...);
uint8_t getSerialLogCount();
void getSerialLogLine(uint8_t idx, char* buffer, size_t bufsize);

// Radar internals
struct RadarTarget {
  bool presence;
  float x, y, speed, distRaw, distanceXY, angleDeg;
};

extern uint8_t           radarBuf[64];
extern uint8_t           radarCount;
extern bool              otaInProgress, startConfigPortal, rebootRequested, serialResetAttempted;
extern unsigned long     rebootRequestedAt;
extern unsigned long     serialResetTime;
extern uint32_t          wifiReconnectCount, radarTimeoutCount, radarSerialRestartCount;
extern bool              wifiReconnectIssued;
extern bool              configPortalActive;
// Debug-Schalter
extern bool              webServerEnabled;
extern bool              otaEnabled;
extern bool              radarSerialRestartEnabled;
extern bool              mqttTelemetryEnabled;
extern const float       ALPHA, RANGE_GATE_SIZE;
extern const uint8_t     multiTargetCmd[12];
extern RadarTarget       smoothed[3];
extern unsigned long     lastSeenTime[3];
extern unsigned long     lastZeroPub;
extern char              g_lastBssid[18];
// Radar frame constants (must be #define for array sizes)
#define RADAR_FRAME_SIZE       30
#define RADAR_TARGET_BLOCKSIZE 8
