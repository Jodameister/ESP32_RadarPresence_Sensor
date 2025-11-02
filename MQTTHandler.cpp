// File: MQTTHandler.cpp

#include "MQTTHandler.h"
#include "Config.h"
#include "RadarHandler.h"

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
    return false;
  }
  return mqttClient.publish(topic, payload);
}

void processMqttCommand(const String& cmd) {
  Serial.print("MQTT CMD: ");
  Serial.println(cmd);

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
    float v = cmd.substring(9).toFloat();
    if (v > 0 && v <= 15) {
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
    publishStatus();
    safePublish(ackTopic, "getStatus OK");
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }
}

void mqttReconnect() {
  static unsigned long lastAttempt = 0;
  const unsigned long RECONNECT_INTERVAL = 5000; // 5 Sekunden zwischen Versuchen

  if (mqttClient.connected()) return;

  unsigned long now = millis();
  if (now - lastAttempt < RECONNECT_INTERVAL) {
    return; // Zu früh für neuen Versuch
  }

  lastAttempt = now;

  Serial.print("MQTT reconnect... ");

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
    Serial.println("OK");

    // SICHERHEIT: Ohne String-Konkatenation
    char cmdTopic[80];
    snprintf(cmdTopic, sizeof(cmdTopic), "%s/cmd", g_mqttTopic.c_str());
    mqttClient.subscribe(cmdTopic);

    // Sofort Status senden
    publishStatus();
  } else {
    Serial.print("FAILED, rc=");
    Serial.println(mqttClient.state());
  }
}