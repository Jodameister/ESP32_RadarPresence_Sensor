// File: RadarHandler.cpp

#include "RadarHandler.h"
#include "Config.h"
#include "MQTTHandler.h"
#include <ArduinoJson.h>

void enableMultiTargetMode() {
  Serial1.write(multiTargetCmd, sizeof(multiTargetCmd));
}

bool readSensorAck(uint16_t expectedCmd, uint32_t timeoutMs) {
  const uint8_t HDR[4] = {0xFD,0xFC,0xFB,0xFA};
  uint32_t start = millis();
  uint8_t buf[20];
  uint8_t idx = 0;
  while (millis() - start < timeoutMs) {
    if (!Serial1.available()) continue;
    buf[idx++] = Serial1.read();
    if (idx >= 8 && memcmp(buf, HDR, 4) == 0) {
      uint16_t len = buf[4] | (buf[5] << 8);
      uint16_t full = 4 + 2 + len + 4;
      if (idx >= full) {
        uint16_t cmd = buf[6] | (buf[7] << 8);
        uint16_t st  = buf[8] | (buf[9] << 8);
        return cmd == expectedCmd && st == 0;
      }
    }
    if (idx >= sizeof(buf)) idx = 0;
  }
  return false;
}

void setMaxRadarRange(float m) {
  g_maxRangeMeters = m;
  uint8_t gate = min((uint8_t)ceil(m / RANGE_GATE_SIZE), (uint8_t)15);

  // Open
  static const uint8_t openCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(openCmd, sizeof(openCmd));
  delay(50);

  // Set
  uint8_t setCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x08,0x00,0x07,0x00,0x01,0x00,
    gate,0x00,0x00,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(setCmd, sizeof(setCmd));
  delay(50);

  // Close
  static const uint8_t closeCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xFE,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(closeCmd, sizeof(closeCmd));
  delay(50);

  bool ok = readSensorAck(0x0007);
  char bufAck[64];
  if (ok) {
    snprintf(bufAck, sizeof(bufAck), "setRange→OK: %.2fm", m);
  } else {
    strcpy(bufAck, "setRange→ERROR");
  }
  mqttClient.publish((g_mqttTopic + "/ack").c_str(), bufAck);
}

void setHoldInterval(uint32_t ms) {
  g_holdIntervalMs = ms;

  static const uint8_t openCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(openCmd, sizeof(openCmd));
  delay(50);

  uint8_t lo = ms & 0xFF, hi = (ms >> 8) & 0xFF;
  uint8_t setCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x08,0x00,0x07,0x00,0x04,0x00,
    lo,hi,0x00,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(setCmd, sizeof(setCmd));
  delay(50);

  static const uint8_t closeCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xFE,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(closeCmd, sizeof(closeCmd));
  delay(50);

  bool ok = readSensorAck(0x0007);
  char bufAck[64];
  if (ok) {
    snprintf(bufAck, sizeof(bufAck), "setHold→OK: %ums", ms);
  } else {
    strcpy(bufAck, "setHold→ERROR");
  }
  mqttClient.publish((g_mqttTopic + "/ack").c_str(), bufAck);
}

void restartRadarSerial() {
  Serial1.end();
  delay(50);
  Serial1.begin(256000, SERIAL_8N1,
                g_radarRxPin.toInt(),
                g_radarTxPin.toInt());
  delay(50);
  setMaxRadarRange(g_maxRangeMeters);
  enableMultiTargetMode();
  mqttClient.publish((g_mqttTopic + "/ack").c_str(), "resetRadar→OK");
}

void parseRadarFrame(const uint8_t* buf, uint8_t len) {
  if (len != RADAR_FRAME_SIZE || memcmp(buf, "\xAA\xFF\x03\x00", 4)) return;
  unsigned long now = millis();
  for (int i = 0; i < 3; i++) {
    const uint8_t* b = buf + 4 + i * RADAR_TARGET_BLOCKSIZE;
    RadarTarget cur; bool seen = false;
    for (int j = 0; j < RADAR_TARGET_BLOCKSIZE; j++) {
      if (b[j]) { seen = true; break; }
    }
    if (seen) {
      lastSeenTime[i] = now;
      int16_t rx = (b[1] & 0x80)
        ? ((b[1] & 0x7F) << 8 | b[0])
        : -((b[1] & 0x7F) << 8 | b[0]);
      int16_t ry = (b[3] & 0x80)
        ? ((b[3] & 0x7F) << 8 | b[2])
        : -((b[3] & 0x7F) << 8 | b[2]);
      cur.presence = true;
      cur.x = rx; cur.y = ry;
      cur.speed = (b[5] & 0x80)
        ? ((b[5] & 0x7F)<<8|b[4])
        : -((b[5] & 0x7F)<<8|b[4]);
      cur.distRaw = (uint16_t)b[6] | ((uint16_t)b[7] << 8);
      cur.distanceXY = sqrtf(rx*rx + ry*ry);
      cur.angleDeg = atan2f(ry, rx) * 180.0f / PI;
    }
    else if (now - lastSeenTime[i] <= g_holdIntervalMs) {
      cur = smoothed[i]; cur.presence = true;
    }
    else {
      cur.presence = false;
      cur.x = cur.y = cur.speed = cur.distRaw = cur.distanceXY = cur.angleDeg = 0;
    }
    if (!smoothed[i].presence) {
      smoothed[i] = cur;
    } else if (cur.presence) {
      smoothed[i].x         = ALPHA * smoothed[i].x + (1 - ALPHA) * cur.x;
      smoothed[i].y         = ALPHA * smoothed[i].y + (1 - ALPHA) * cur.y;
      smoothed[i].speed     = ALPHA * smoothed[i].speed + (1 - ALPHA) * cur.speed;
      smoothed[i].distRaw   = cur.distRaw;
      smoothed[i].distanceXY= sqrtf(smoothed[i].x*smoothed[i].x +
                                   smoothed[i].y*smoothed[i].y);
      smoothed[i].angleDeg  = atan2f(smoothed[i].y, smoothed[i].x) * 180.0f / PI;
    }
    smoothed[i].presence = cur.presence;
  }
}

void readRadarData() {
  while (Serial1.available()) {
    radarBuf[radarCount++] = Serial1.read();
    if (radarCount >= RADAR_FRAME_SIZE) {
      if (radarBuf[radarCount-2]==0x55 && radarBuf[radarCount-1]==0xCC) {
        parseRadarFrame(radarBuf, radarCount);
      }
      radarCount = 0;
    }
  }
}

void publishRadarJson() {
  StaticJsonDocument<256> doc;
  int cnt = 0; for (auto &t : smoothed) if (t.presence) cnt++;
  doc["targetCount"] = cnt;
  for (int i = 0; i < 3; i++) {
    char key[10]; snprintf(key,10,"target%d",i+1);
    auto o = doc.createNestedObject(key);
    if (!smoothed[i].presence) {
      o["presence"] = false;
    } else {
      o["presence"] = true;
      o["x"]        = round(smoothed[i].x);
      o["y"]        = round(smoothed[i].y);
      o["speed"]    = round(smoothed[i].speed);
      o["distRaw"]  = round(smoothed[i].distRaw);
      o["distance"] = round(smoothed[i].distanceXY);
      o["angleDeg"] = round(smoothed[i].angleDeg);
    }
  }
  char bufOut[256]; serializeJson(doc, bufOut);
  mqttClient.publish(g_mqttTopic.c_str(), bufOut);
}

void publishStatus() {
  StaticJsonDocument<256> doc;
  doc["fwVersion"] = FW_VERSION;
  doc["uptime_min"] = millis()/60000;
  doc["rssi"]       = WiFi.RSSI();
  doc["heap"]       = ESP.getFreeHeap();
  doc["holdMs"]     = g_holdIntervalMs;
  doc["range_m"]    = g_maxRangeMeters;
  char bufOut[256]; serializeJson(doc, bufOut);
  mqttClient.publish((g_mqttTopic + "/status").c_str(), bufOut);
}

void checkRadarConnection() {
  if (millis() - lastRadarDataTime > NO_DATA_TIMEOUT) {
    if (!serialResetAttempted) {
      restartRadarSerial();
      serialResetAttempted = true;
      serialResetTime     = millis();
      radarTimeoutCount++;
    }
    else if (millis() - serialResetTime > RESTART_TIMEOUT) {
      ESP.restart();
    }
  } else {
    serialResetAttempted = false;
  }
}