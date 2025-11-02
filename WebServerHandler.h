// File: WebServerHandler.h

#pragma once

#include <WebServer.h>

void setupWebServer();
void handleWebServer();
void sendRadarData();

extern WebServer webServer;
