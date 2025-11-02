// File: MQTTHandler.cpp

#include "MQTTHandler.h"
#include "Config.h"
#include "RadarHandler.h"

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();
  processMqttCommand(msg);
}

void processMqttCommand(const String& cmd) {
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
    setMaxRadarRange(v);
  }
  else if (cmd.startsWith("setHold:")) {
    uint32_t v = cmd.substring(8).toInt();
    setHoldInterval(v);
  }
  else if (cmd == "getStatus") {
    publishStatus();
    mqttClient.publish((g_mqttTopic + "/ack").c_str(), "getStatus OK");
  }
}

void mqttReconnect() {
  if (!mqttClient.connected()) {
    while (!mqttClient.connected()) {
      String id = "RD03D-" + String(random(0xffff), HEX);
      if (mqttClient.connect(id.c_str())) {
        mqttClient.subscribe((g_mqttTopic + "/cmd").c_str());
      } else {
        delay(2000);
      }
    }
  }
}