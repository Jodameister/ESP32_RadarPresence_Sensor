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

void processMqttCommand(const String& cmd) {
  Serial.print("MQTT CMD: ");
  Serial.println(cmd);

  if (cmd == "config") {
    mqttClient.publish((g_mqttTopic + "/ack").c_str(), "config OK");
    startConfigPortal = true;
  }
  else if (cmd == "reboot") {
    mqttClient.publish((g_mqttTopic + "/ack").c_str(), "reboot OK");
    rebootRequested = true;
  }
  else if (cmd == "resetRadar") {
    restartRadarSerial();
  }
  else if (cmd.startsWith("setRange:")) {
    float v = cmd.substring(9).toFloat();
    if (v > 0 && v <= 15) {  // SICHERHEIT: Validierung
      setMaxRadarRange(v);
    } else {
      mqttClient.publish((g_mqttTopic + "/ack").c_str(), "setRange ERROR: invalid value");
    }
  }
  else if (cmd.startsWith("setHold:")) {
    uint32_t v = cmd.substring(8).toInt();
    if (v >= 0 && v <= 10000) {  // SICHERHEIT: Max 10 Sekunden
      setHoldInterval(v);
    } else {
      mqttClient.publish((g_mqttTopic + "/ack").c_str(), "setHold ERROR: invalid value");
    }
  }
  else if (cmd == "getStatus") {
    publishStatus();
    mqttClient.publish((g_mqttTopic + "/ack").c_str(), "getStatus OK");
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
  String id = "RD03D-" + String(random(0xffff), HEX);

  // SICHERHEIT: Mit Last Will Testament
  // connect(clientId, willTopic, willQoS, willRetain, willMessage)
  String willTopic = g_mqttTopic + "/status";
  bool connected = mqttClient.connect(
    id.c_str(),
    willTopic.c_str(),
    0,                           // QoS 0
    true,                        // Retain
    "{\"status\":\"offline\"}"   // LWT Message
  );

  if (connected) {
    Serial.println("OK");
    mqttClient.subscribe((g_mqttTopic + "/cmd").c_str());

    // Sofort Status senden
    publishStatus();
  } else {
    Serial.print("FAILED, rc=");
    Serial.println(mqttClient.state());
  }
}