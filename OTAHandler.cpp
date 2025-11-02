// File: OTAHandler.cpp

#include "Config.h"
#include <ArduinoOTA.h>

void otaSetup() {
  ArduinoOTA.onStart([](){ otaInProgress = true; });
  ArduinoOTA.onEnd(  [](){ otaInProgress = false; });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t){
    Serial.printf("OTA %u%%\n", (p*100)/t);
  });
  ArduinoOTA.setHostname(g_host.c_str());
  ArduinoOTA.setPassword(g_otaPass.c_str());
  ArduinoOTA.begin();
}