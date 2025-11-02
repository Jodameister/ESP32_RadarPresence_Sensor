// File: WebServerHandler.h

#pragma once

#include <WebServer.h>

void setupWebServer();
void handleWebServer();
void stopWebServer();
bool isWebServerRunning();
void sendRadarData();

extern WebServer webServer;
