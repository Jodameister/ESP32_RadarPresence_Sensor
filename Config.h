// File: Config.h

#pragma once

#include <Preferences.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Version
#define FW_VERSION "v1."

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
extern const unsigned long RADAR_INTERVAL_MS, STATUS_INTERVAL, NO_DATA_TIMEOUT, RESTART_TIMEOUT;
extern const int          RADAR_BOOT_PIN;

// Forward declarations
void setupWiFiManager();
void saveParamCallback();   // <<< neu

// Radar internals
struct RadarTarget {
  bool presence;
  float x, y, speed, distRaw, distanceXY, angleDeg;
};

extern uint8_t           radarBuf[64];
extern uint8_t           radarCount;
extern bool              otaInProgress, startConfigPortal, rebootRequested, serialResetAttempted;
extern unsigned long     serialResetTime;
extern uint32_t          wifiReconnectCount, radarTimeoutCount;
extern const float       ALPHA, RANGE_GATE_SIZE;
extern const uint8_t     multiTargetCmd[12];
extern RadarTarget       smoothed[3];
extern unsigned long     lastSeenTime[3];
extern unsigned long     lastZeroPub;
// Radar frame constants (must be #define for array sizes)
#define RADAR_FRAME_SIZE       30
#define RADAR_TARGET_BLOCKSIZE 8