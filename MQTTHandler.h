// File: MQTTHandler.h

#pragma once
#include "Config.h"
#include <Arduino.h>

void mqttCallback(char* topic, byte* payload, unsigned int length);
void processMqttCommand(const String& cmd);
void mqttReconnect();
bool safePublish(const char* topic, const char* payload);