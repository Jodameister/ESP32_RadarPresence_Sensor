// File: RadarHandler.cpp

#include "RadarHandler.h"
#include "Config.h"
#include "MQTTHandler.h"
#include "WebServerHandler.h"
#include <ArduinoJson.h>
#include <esp_system.h>

static const char* resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:    return "power-on";
    case ESP_RST_EXT:        return "external";
    case ESP_RST_SW:         return "software";
    case ESP_RST_PANIC:      return "panic";
    case ESP_RST_INT_WDT:    return "int-watchdog";
    case ESP_RST_TASK_WDT:   return "task-watchdog";
    case ESP_RST_WDT:        return "watchdog";
    case ESP_RST_DEEPSLEEP:  return "deep-sleep";
    case ESP_RST_BROWNOUT:   return "brown-out";
    case ESP_RST_SDIO:       return "sdio";
    default:                 return "unknown";
  }
}

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
        uint16_t cmdRaw = buf[6] | (buf[7] << 8);
        uint16_t st     = buf[8] | (buf[9] << 8);
        uint16_t cmd    = cmdRaw & 0x00FF;
        if (cmd == expectedCmd || cmdRaw == expectedCmd) {
          if (st != 0) {
            logPrintf("Radar ACK 0x%04X status=%u\n", cmdRaw, st);
          }
          return st == 0;
        }
        // Open/Close-ACKs können vor dem erwarteten SET kommen → still ignorieren
        if (cmdRaw == 0x01FF || cmdRaw == 0x01FE) {
          idx = 0;
          continue;
        }
        // Unerwartetes ACK verwerfen und weiter warten
        logPrintf("Radar ACK unexpected cmd=0x%04X expecting 0x%04X\n", cmdRaw, expectedCmd);
        idx = 0;
      }
    }
    if (idx >= sizeof(buf)) idx = 0;
  }
  return false;
}

void setMaxRadarRange(float m) {
  while (Serial1.available()) {
    Serial1.read();
  }

  g_maxRangeMeters = m;
  uint8_t gate = min((uint8_t)ceil(m / RANGE_GATE_SIZE), (uint8_t)15);

  // Open
  static const uint8_t openCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(openCmd, sizeof(openCmd));
  delayMicroseconds(RADAR_CMD_DELAY_US);

  // Set
  uint8_t setCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x08,0x00,0x07,0x00,0x01,0x00,
    gate,0x00,0x00,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(setCmd, sizeof(setCmd));
  delayMicroseconds(RADAR_CMD_DELAY_US);

  // Close
  static const uint8_t closeCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xFE,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(closeCmd, sizeof(closeCmd));
  delayMicroseconds(RADAR_CMD_DELAY_US);

  bool ok = readSensorAck(0x0007);
  char bufAck[64];
  if (ok) {
    snprintf(bufAck, sizeof(bufAck), "setRange→OK: %.2fm", m);
  } else {
    strcpy(bufAck, "setRange→ERROR");
  }

  char topic[MQTT_TOPIC_BUFFER_SIZE];
  buildMqttTopic("ack", topic, sizeof(topic));
  safePublish(topic, bufAck);
}

void setHoldInterval(uint32_t ms) {
  while (Serial1.available()) {
    Serial1.read();
  }

  g_holdIntervalMs = ms;

  static const uint8_t openCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(openCmd, sizeof(openCmd));
  delayMicroseconds(RADAR_CMD_DELAY_US);

  uint8_t lo = ms & 0xFF, hi = (ms >> 8) & 0xFF;
  uint8_t setCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x08,0x00,0x07,0x00,0x04,0x00,
    lo,hi,0x00,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(setCmd, sizeof(setCmd));
  delayMicroseconds(RADAR_CMD_DELAY_US);

  static const uint8_t closeCmd[] = {
    0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xFE,0x00,0x04,0x03,0x02,0x01
  };
  Serial1.write(closeCmd, sizeof(closeCmd));
  delayMicroseconds(RADAR_CMD_DELAY_US);

  bool ok = readSensorAck(0x0007);
  char bufAck[64];
  if (ok) {
    snprintf(bufAck, sizeof(bufAck), "setHold→OK: %ums", ms);
  } else {
    strcpy(bufAck, "setHold→ERROR");
  }

  char topic[MQTT_TOPIC_BUFFER_SIZE];
  buildMqttTopic("ack", topic, sizeof(topic));
  safePublish(topic, bufAck);
}

void restartRadarSerial() {
  logPrintln("Restarting radar serial...");
  radarSerialRestartCount++;

  // SICHERHEIT: Buffer leeren BEVOR Serial1.end()
  while (Serial1.available()) {
    Serial1.read();
  }

  // SICHERHEIT: radarCount zurücksetzen
  radarCount = 0;

  Serial1.end();

  // Non-blocking delay mit MQTT-Loop
  unsigned long start = millis();
  while (millis() - start < 100) {
    mqttClient.loop();
    yield();
  }

  Serial1.begin(256000, SERIAL_8N1,
                g_radarRxPin.toInt(),
                g_radarTxPin.toInt());

  // Non-blocking delay mit MQTT-Loop
  start = millis();
  while (millis() - start < 100) {
    mqttClient.loop();
    yield();
  }

  setMaxRadarRange(g_maxRangeMeters);
  enableMultiTargetMode();

  // SICHERHEIT: lastRadarDataTime aktualisieren
  lastRadarDataTime = millis();

  char topic[MQTT_TOPIC_BUFFER_SIZE];
  buildMqttTopic("ack", topic, sizeof(topic));
  safePublish(topic, "resetRadar→OK");
  logPrintln("Radar serial restarted");
}

void parseRadarFrame(const uint8_t* buf, uint8_t len) {
  // SICHERHEIT: Strikte Validierung
  if (!buf || len != RADAR_FRAME_SIZE) return;
  if (buf[0] != 0xAA || buf[1] != 0xFF || buf[2] != 0x03 || buf[3] != 0x00) return;

  unsigned long now = millis();
  for (int i = 0; i < 3; i++) {
    int offset = 4 + i * RADAR_TARGET_BLOCKSIZE;

    // SICHERHEIT: Bounds-Check
    if (offset + RADAR_TARGET_BLOCKSIZE > len) {
      logPrint("WARN: parseRadarFrame offset out of bounds: ");
      logPrintln(String(offset));
      break;
    }

    const uint8_t* b = buf + offset;

    // SICHERHEIT: cur initialisieren!
    RadarTarget cur = {0};
    bool seen = false;

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
      cur.x = rx;
      cur.y = ry;
      cur.speed = (b[5] & 0x80)
        ? ((b[5] & 0x7F)<<8|b[4])
        : -((b[5] & 0x7F)<<8|b[4]);
      cur.distRaw = (uint16_t)b[6] | ((uint16_t)b[7] << 8);
      cur.distanceXY = sqrtf(rx*rx + ry*ry);
      cur.angleDeg = atan2f(ry, rx) * 180.0f / PI;
    }
    else if (now - lastSeenTime[i] <= g_holdIntervalMs) {
      cur = smoothed[i];
      cur.presence = true;
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
  const uint16_t MAX_READ = 256;
  uint16_t readCnt = 0;
  static uint8_t lastF[RADAR_FRAME_SIZE];

  while (Serial1.available() && readCnt < MAX_READ) {
    lastRadarDataTime = millis();

    // SICHERHEIT: Buffer-Overflow-Schutz BEVOR wir schreiben
    if (radarCount >= sizeof(radarBuf)) {
      radarCount = 0;
      logPrintln("WARN: radarBuf overflow reset");
    }

    uint8_t byte = Serial1.read();

    // Sync-Optimierung: Bei leerem Buffer nur auf Start-Marker warten
    if (radarCount == 0 && byte != 0xAA) {
      readCnt++;
      continue; // Warte auf Frame-Start
    }

    radarBuf[radarCount] = byte;
    radarCount++;
    readCnt++;

    // Frame-Ende erkannt?
    if (radarCount >= 2 &&
        radarBuf[radarCount-2] == 0x55 &&
        radarBuf[radarCount-1] == 0xCC) {

      // SICHERHEIT: Nur gültige Frames verarbeiten
      if (radarCount == RADAR_FRAME_SIZE) {
        // Prüfen, ob das Frame echte Targets enthält:
        bool hasAnyTarget = false;
        for (int i = 0; i < 3; i++) {
          int offset = 4 + i * RADAR_TARGET_BLOCKSIZE;

          // SICHERHEIT: Bounds-Check vor Zugriff
          if (offset + RADAR_TARGET_BLOCKSIZE > RADAR_FRAME_SIZE) {
            break;
          }

          const uint8_t* blk = radarBuf + offset;
          bool empty = true;
          for (int j = 0; j < RADAR_TARGET_BLOCKSIZE; j++) {
            if (blk[j]) { empty = false; break; }
          }
          if (!empty) {
            hasAnyTarget = true;
            break;
          }
        }

        // Parsing auslösen, wenn
        //    a) es neue Target-Daten sind (memcmp ≠ 0) oder
        //    b) gar keine Targets mehr da sind (!hasAnyTarget)
        if (memcmp(radarBuf, lastF, RADAR_FRAME_SIZE) != 0
            || !hasAnyTarget) {
          memcpy(lastF, radarBuf, RADAR_FRAME_SIZE);
          parseRadarFrame(radarBuf, radarCount);
        }
      } else if (radarCount > RADAR_FRAME_SIZE) {
        // SICHERHEIT: Ungültiger Frame zu lang
        logPrint("WARN: Invalid frame size: ");
        logPrintln(String(radarCount));
      }

      radarCount = 0;
    }
  }

  if (Serial1.available()) yield();
}

void publishRadarJson() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<512> doc;
  int cnt = 0;
  for (auto &t: smoothed) if (t.presence) cnt++;
  doc["targetCount"] = cnt;
  for (int i = 0; i < 3; i++) {
    char key[12];
    snprintf(key, sizeof(key), "target%d", i + 1);
    auto o = doc.createNestedObject(key);
    if (!smoothed[i].presence) {
      o["presence"] = false;
    } else {
      o["presence"]  = true;
      o["x"]         = round(smoothed[i].x);
      o["y"]         = round(smoothed[i].y);
      o["speed"]     = round(smoothed[i].speed);
      o["distRaw"]   = round(smoothed[i].distRaw);
      o["distance"]  = round(smoothed[i].distanceXY);
      o["angleDeg"]  = round(smoothed[i].angleDeg);
    }
  }
  char buf[512];
  serializeJson(doc, buf);
  unsigned long now = millis();

  if (!cnt) {
    // 0 Targets: max. 1× pro Sekunde
    if (now - lastZeroPub < 1000) return;
    lastZeroPub = now;
  }

  if (!safePublish(g_mqttTopic.c_str(), buf)) {
    logPrintln("WARN: MQTT publish radar failed");
  }
}

void publishStatus() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<512> doc;
  doc["fwVersion"]      = FW_VERSION;
  doc["uptime_min"]     = millis()/60000;
  char uptimeStr[8];
  formatUptime(uptimeStr, sizeof(uptimeStr));
  doc["uptime"]         = uptimeStr;
  doc["resetReason"]    = resetReasonToString(esp_reset_reason());
  doc["ip"]             = WiFi.localIP().toString();
  doc["rssi"]           = WiFi.RSSI();
  doc["channel"]        = WiFi.channel();
  doc["heap_free"]      = ESP.getFreeHeap();
#ifdef CONFIG_FREERTOS_USE_TRACE_FACILITY
  doc["psram_free"]     = ESP.getFreePsram();
#endif
  doc["temp_c"]         = temperatureRead();
  doc["mqttState"]      = mqttClient.state();
  doc["wifiReconnects"] = wifiReconnectCount;
  doc["radarTimeouts"]  = radarTimeoutCount;
  doc["radarSerialRestarts"] = radarSerialRestartCount;
  doc["lastRadarDelta"] = millis() - lastRadarDataTime;
  doc["holdMs"]         = g_holdIntervalMs;
  doc["range_m"]        = g_maxRangeMeters;
  doc["webServer"]      = isWebServerRunning();

  // Warnmeldungen hinzufügen
  JsonArray warnings = doc.createNestedArray("warnings");
  if (WiFi.RSSI() < -80) {
    warnings.add("Schwaches WiFi-Signal");
  }
  if (ESP.getFreeHeap() < 10000) {
    warnings.add("Wenig freier Heap");
  }
  if (radarTimeoutCount > 0) {
    warnings.add("Radar-Timeouts erkannt");
  }
  if (millis() - lastRadarDataTime > NO_DATA_TIMEOUT) {
    warnings.add("Keine Radar-Daten");
  }

  char buf[512];
  serializeJson(doc, buf);

  char statusTopic[MQTT_TOPIC_BUFFER_SIZE];
  buildMqttTopic("status", statusTopic, sizeof(statusTopic));

  if (!safePublishRetain(statusTopic, buf)) {
    logPrintln("WARN: MQTT publish status failed");
  } else {
    logPrintln("Status published");
  }
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
