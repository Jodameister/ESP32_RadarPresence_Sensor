// File: MQTTHandler.cpp

#include "MQTTHandler.h"
#include <string.h>
#include "Config.h"
#include "RadarHandler.h"
#include "WebServerHandler.h"

static void logMqttDiag(const char* prefix, const char* topic, const char* payload, bool retain) {
  size_t len = payload ? strlen(payload) : 0;
  logPrintf("%s topic=%s len=%u retain=%d\n",
            prefix,
            topic ? topic : "(null)",
            static_cast<unsigned int>(len),
            retain ? 1 : 0);
  logPrintf("MQTT state=%d connected=%d\n", mqttClient.state(), mqttClient.connected() ? 1 : 0);
  logPrintf("WiFi status=%d RSSI=%d CH=%d BSSID=%s IP=%s\n",
            WiFi.status(),
            WiFi.RSSI(),
            WiFi.channel(),
            (g_lastBssid[0] != '\0') ? g_lastBssid : "n/a",
            WiFi.localIP().toString().c_str());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // SICHERHEIT: Effizienter ohne String-Konkatenation
  if (length == 0 || length > 128) return; // Maximal 128 Zeichen

  char msg[129];
  memcpy(msg, payload, length);
  msg[length] = '\0';

  // Trim whitespace
  char* start = msg;
  while (*start == ' ' || *start == '\t' || *start == '\n' || *start == '\r') {
    start++;
  }

  processMqttCommand(String(start));
}

bool safePublish(const char* topic, const char* payload) {
  if (!mqttClient.connected()) {
    logMqttDiag("MQTT publish blocked (disconnected)", topic, payload, false);
    return false;
  }
  if (!mqttClient.publish(topic, payload)) {
    logMqttDiag("MQTT publish post (failed)", topic, payload, false);
    return false;
  }
  return true;
}

bool safePublishRetain(const char* topic, const char* payload) {
  if (!mqttClient.connected()) {
    logMqttDiag("MQTT retain publish blocked (disconnected)", topic, payload, true);
    return false;
  }
  if (!mqttClient.publish(topic, payload, true)) {
    logMqttDiag("MQTT retain publish post (failed)", topic, payload, true);
    return false;
  }
  return true;
}

void processMqttCommand(const String& cmd) {
  logPrint("MQTT CMD: ");
  logPrintln(cmd);

  char ackTopic[MQTT_TOPIC_BUFFER_SIZE];
  buildMqttTopic("ack", ackTopic, sizeof(ackTopic));

  if (cmd == "config") {
    safePublish(ackTopic, "config OK");
    startConfigPortal = true;
  }
  else if (cmd == "reboot") {
    safePublish(ackTopic, "reboot OK");
    rebootRequested = true;
  }
  else if (cmd == "resetRadar") {
    restartRadarSerial();
  }
  else if (cmd.startsWith("setRange:")) {
    String val = cmd.substring(9);
    if (val.length() == 0) {
      safePublish(ackTopic, "setRange ERROR: invalid value");
      return;
    }
    char* endPtr = nullptr;
    float v = strtof(val.c_str(), &endPtr);
    if (endPtr == val.c_str() || *endPtr != '\0') {
      safePublish(ackTopic, "setRange ERROR: invalid value");
    } else if (v > 0.5f && v <= 15.0f) {
      setMaxRadarRange(v);
    } else {
      safePublish(ackTopic, "setRange ERROR: invalid value");
    }
  }
  else if (cmd.startsWith("setHold:")) {
    uint32_t v = cmd.substring(8).toInt();
    if (v >= 0 && v <= 10000) {
      setHoldInterval(v);
    } else {
      safePublish(ackTopic, "setHold ERROR: invalid value");
    }
  }
  else if (cmd == "getStatus") {
    if (!mqttTelemetryEnabled) {
      safePublish(ackTopic, "getStatus ERROR: telemetry disabled");
    } else {
      publishStatus();
      safePublish(ackTopic, "getStatus OK");
    }
  }
  else if (cmd == "webServer:on") {
    if (!webServerEnabled) {
      safePublish(ackTopic, "webServer ERROR: disabled");
    } else if (configPortalActive) {
      safePublish(ackTopic, "webServer ERROR: config portal active");
    } else {
      setupWebServer();
      safePublish(ackTopic, "webServer ON");
    }
  }
  else if (cmd == "webServer:off") {
    if (!webServerEnabled) {
      safePublish(ackTopic, "webServer ERROR: disabled");
    } else {
      stopWebServer();
      safePublish(ackTopic, "webServer OFF");
    }
  }
  else if (cmd == "help") {
    const char* helpMsg =
      "Available commands:\n"
      "config - Start WiFi config portal\n"
      "reboot - Restart ESP32\n"
      "resetRadar - Restart radar serial\n"
      "setRange:<value> - Set max range (0-15m)\n"
      "setHold:<value> - Set hold interval (0-10000ms)\n"
      "getStatus - Publish current status\n"
      "webServer:on - Start HTTP status server\n"
      "webServer:off - Stop HTTP status server\n"
      "help - Show this help";
    if (!webServerEnabled) {
      safePublish(ackTopic, "Hinweis: WebServer ist aktuell deaktiviert");
    }
    safePublish(ackTopic, helpMsg);
  }
  else {
    logPrint("Unknown command: ");
    logPrintln(cmd);
    safePublish(ackTopic, "ERROR: Unknown command. Send 'help' for available commands.");
  }
}

void mqttReconnect() {
  static unsigned long lastAttempt = 0;
  static uint32_t attemptCount = 0;
  const unsigned long RECONNECT_INTERVAL = 5000; // 5 Sekunden zwischen Versuchen

  if (WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) return;

  unsigned long now = millis();
  if (now - lastAttempt < RECONNECT_INTERVAL) {
    return; // Zu früh für neuen Versuch
  }

  lastAttempt = now;
  attemptCount++;

  logPrintf("MQTT reconnect #%lu... ", attemptCount);
  logPrintf("WiFi status=%d RSSI=%d CH=%d BSSID=%s IP=%s\n",
            WiFi.status(),
            WiFi.RSSI(),
            WiFi.channel(),
            (g_lastBssid[0] != '\0') ? g_lastBssid : "n/a",
            WiFi.localIP().toString().c_str());

  // SICHERHEIT: Ohne String-Konkatenation
  char id[20];
  snprintf(id, sizeof(id), "RD03D-%04X", random(0xffff));

  char willTopic[80];
  snprintf(willTopic, sizeof(willTopic), "%s/status", g_mqttTopic.c_str());

  // SICHERHEIT: Mit Last Will Testament
  bool connected = mqttClient.connect(
    id,
    willTopic,
    0,                           // QoS 0
    true,                        // Retain
    "{\"status\":\"offline\"}"   // LWT Message
  );

  if (connected) {
    logPrintf("MQTT connected OK id=%s host=%s:%s\n",
              id, g_mqttServer.c_str(), g_mqttPort.c_str());

    // SICHERHEIT: Ohne String-Konkatenation
    char cmdTopic[80];
    snprintf(cmdTopic, sizeof(cmdTopic), "%s/cmd", g_mqttTopic.c_str());
    mqttClient.subscribe(cmdTopic);

    // Sofort Status senden
    publishStatus();
  } else {
    logPrintf("MQTT connect FAILED rc=%d\n", mqttClient.state());
    logMqttDiag("MQTT connect failed diag", g_mqttTopic.c_str(), nullptr, false);
  }
}
