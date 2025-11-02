// File: RadarHandler.h

#pragma once
#include "Config.h"

void enableMultiTargetMode();
bool readSensorAck(uint16_t expectedCmd, uint32_t timeoutMs = 200);
void setMaxRadarRange(float meters);
void setHoldInterval(uint32_t ms);
void restartRadarSerial();

void readRadarData();
void parseRadarFrame(const uint8_t* buf, uint8_t len);
void checkRadarConnection();

void publishRadarJson();
void publishStatus();