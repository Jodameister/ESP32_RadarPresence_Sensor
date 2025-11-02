// File: WebServerHandler.cpp
// Uses standard ESP32 WebServer - no external libraries needed

#include "WebServerHandler.h"
#include "Config.h"
#include "RadarHandler.h"
#include "MQTTHandler.h"
#include <ArduinoJson.h>
#include <esp_system.h>

WebServer webServer(80);

// Track SSE clients
struct SSEClient {
  WiFiClient client;
  unsigned long lastPing;
  bool active;
};

SSEClient sseClients[1];
const int MAX_SSE_CLIENTS = 1;

// Embedded HTML page
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Radar Monitor</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }

    body {
      font-family: -apple-system, BlinkMacSystemFont, "SF Pro Display", "SF Pro Text", "Segoe UI", Arial, sans-serif;
      background: #000000;
      color: #f5f5f7;
      margin: 0;
      padding: 12px 20px;
      min-height: 100vh;
      -webkit-font-smoothing: antialiased;
      -moz-osx-font-smoothing: grayscale;
    }

    h1 {
      background: linear-gradient(90deg, #00d4ff, #0066ff);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      background-clip: text;
      text-align: center;
      margin-bottom: 20px;
      font-size: 32px;
      font-weight: 700;
      letter-spacing: -0.5px;
    }

    #status {
      padding: 10px 20px;
      border-radius: 20px;
      margin: 0 auto 25px;
      font-size: 14px;
      text-align: center;
      max-width: 200px;
      font-weight: 500;
      backdrop-filter: blur(20px);
    }
    #status.connected {
      background: rgba(52, 199, 89, 0.15);
      border: 1px solid rgba(52, 199, 89, 0.3);
      color: #34c759;
    }
    #status.disconnected {
      background: rgba(255, 69, 58, 0.15);
      border: 1px solid rgba(255, 69, 58, 0.3);
      color: #ff453a;
    }

    /* Dashboard Grid */
    #dashboard {
      display: grid;
      grid-template-columns: 280px 1fr;
      grid-template-rows: auto auto auto;
      gap: 15px;
      max-width: 1600px;
      margin: 0 auto;
    }

    /* Card Base Style - Apple Glassmorphism */
    .card {
      background: rgba(29, 29, 31, 0.72);
      backdrop-filter: saturate(180%) blur(20px);
      border-radius: 18px;
      border: 1px solid rgba(255, 255, 255, 0.1);
      padding: 24px;
      transition: all 0.3s ease;
    }
    .card:hover {
      border-color: rgba(255, 255, 255, 0.15);
      box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
    }
    .card h3 {
      font-size: 17px;
      font-weight: 600;
      margin-bottom: 16px;
      color: #f5f5f7;
      letter-spacing: -0.3px;
    }

    /* ESP Info Box */
    #esp-info {
      grid-row: 1;
      background: linear-gradient(135deg, rgba(0, 122, 255, 0.1), rgba(52, 199, 89, 0.1));
    }

    /* Radar Canvas */
    canvas {
      grid-row: 1;
      grid-column: 2;
      background: rgba(0, 0, 0, 0.4);
      border-radius: 12px;
      width: 100%;
      height: 100%;
      border: 1px solid rgba(255, 255, 255, 0.05);
    }

    /* Control Buttons */
    #control-buttons {
      display: flex;
      flex-direction: column;
      gap: 12px;
      grid-row: 2;
    }

    /* Radar Settings */
    #radar-settings {
      grid-row: 3;
      background: linear-gradient(135deg, rgba(255, 204, 0, 0.1), rgba(255, 149, 0, 0.1));
    }

    /* Target Info */
    #target-info {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 15px;
      grid-row: 2;
      grid-column: 2;
    }

    /* Warnings */
    #warnings {
      grid-row: 3;
      grid-column: 2;
      background: linear-gradient(135deg, rgba(0, 122, 255, 0.1), rgba(10, 132, 255, 0.1));
    }
    /* Apple Buttons */
    .btn {
      padding: 14px 20px;
      border: none;
      border-radius: 12px;
      font-size: 15px;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.2s cubic-bezier(0.4, 0, 0.2, 1);
      width: 100%;
      text-align: center;
      letter-spacing: -0.2px;
      position: relative;
      overflow: hidden;
    }
    .btn::before {
      content: '';
      position: absolute;
      top: 50%;
      left: 50%;
      width: 0;
      height: 0;
      border-radius: 50%;
      background: rgba(255, 255, 255, 0.1);
      transform: translate(-50%, -50%);
      transition: width 0.6s, height 0.6s;
    }
    .btn:hover::before {
      width: 300px;
      height: 300px;
    }
    .btn-danger {
      background: linear-gradient(135deg, #ff453a 0%, #ff2d55 100%);
      color: white;
      box-shadow: 0 4px 12px rgba(255, 69, 58, 0.3);
    }
    .btn-danger:hover {
      box-shadow: 0 6px 20px rgba(255, 69, 58, 0.4);
      transform: translateY(-1px);
    }
    .btn-warning {
      background: linear-gradient(135deg, #ff9f0a 0%, #ff9500 100%);
      color: white;
      box-shadow: 0 4px 12px rgba(255, 159, 10, 0.3);
    }
    .btn-warning:hover {
      box-shadow: 0 6px 20px rgba(255, 159, 10, 0.4);
      transform: translateY(-1px);
    }

    /* Info Items */
    .info-item {
      margin: 10px 0;
      font-size: 14px;
      display: flex;
      justify-content: space-between;
      padding: 8px 0;
      border-bottom: 1px solid rgba(255, 255, 255, 0.05);
    }
    .info-item:last-child {
      border-bottom: none;
    }
    .info-label {
      font-weight: 500;
      color: rgba(245, 245, 247, 0.6);
    }
    .info-value {
      color: #0a84ff;
      font-weight: 600;
    }

    /* Target Boxes */
    .target-box {
      background: linear-gradient(135deg, rgba(255, 159, 10, 0.1), rgba(255, 204, 0, 0.1));
    }
    .target-data {
      font-size: 13px;
      line-height: 1.6;
    }
    .target-data div {
      margin: 8px 0;
      display: flex;
      justify-content: space-between;
      padding: 6px 0;
      border-bottom: 1px solid rgba(255, 255, 255, 0.05);
    }
    .target-data div:last-child {
      border-bottom: none;
    }
    .target-data strong {
      color: rgba(245, 245, 247, 0.6);
      font-weight: 500;
    }

    /* Warning Items */
    .warning-item {
      background: rgba(255, 159, 10, 0.1);
      border-left: 3px solid #ff9f0a;
      padding: 12px 16px;
      border-radius: 8px;
      margin: 8px 0;
      font-size: 14px;
      font-weight: 500;
    }

    @media (max-width: 1024px) {
      #dashboard {
        grid-template-columns: 1fr;
        grid-template-rows: auto;
      }
      #radar-container {
        grid-row: 2;
      }
      #target-info {
        grid-template-columns: 1fr;
        grid-column: 1;
      }
      #warnings {
        grid-column: 1;
      }
    }
  </style>
</head>
<body>
  <h1>🎯 Radar Live Monitor</h1>
  <div id="status" class="connected">Verbunden</div>

  <div id="dashboard">
    <!-- ESP Infos -->
    <div id="esp-info" class="card">
      <h3>ESP Infos</h3>
      <div class="info-item"><span class="info-label">Firmware:</span> <span id="fwVersion" class="info-value">-</span></div>
      <div class="info-item"><span class="info-label">Reset Reason:</span> <span id="resetReason" class="info-value">-</span></div>
      <div class="info-item"><span class="info-label">Temperatur:</span> <span id="temperature" class="info-value">-</span></div>
      <div class="info-item"><span class="info-label">Radar Timeouts:</span> <span id="radarTimeouts" class="info-value">0</span></div>
      <div class="info-item"><span class="info-label">Radar Restarts:</span> <span id="radarSerialRestarts" class="info-value">0</span></div>
      <div class="info-item"><span class="info-label">IP:</span> <span id="ip" class="info-value">-</span></div>
      <div class="info-item"><span class="info-label">Uptime:</span> <span id="uptime" class="info-value">0 min</span></div>
      <div class="info-item"><span class="info-label">RSSI:</span> <span id="rssi" class="info-value">0 dBm</span></div>
      <div class="info-item"><span class="info-label">Heap:</span> <span id="heap" class="info-value">0 KB</span></div>
      <div class="info-item"><span class="info-label">Targets:</span> <span id="targetCount" class="info-value">0</span></div>
    </div>

    <!-- Radar Visualization -->
    <canvas id="radar" width="800" height="450"></canvas>

    <!-- Control Buttons -->
    <div id="control-buttons">
      <button class="btn btn-danger" onclick="if(confirm('ESP32 wirklich neustarten?')) sendCommand('reboot')">Restart ESP</button>
      <button class="btn btn-danger" onclick="sendCommand('resetRadar')">Restart Radar</button>
      <button class="btn btn-warning" onclick="sendCommand('config')">WiFiManager start</button>
    </div>

    <!-- Radar Settings -->
    <div id="radar-settings" class="card">
      <h3>Radar Setting Information</h3>
      <div class="info-item"><span class="info-label">Range:</span> <span id="maxRange" class="info-value">0m</span></div>
      <div class="info-item"><span class="info-label">Hold:</span> <span id="holdMs" class="info-value">0ms</span></div>
    </div>

    <!-- Target Informations -->
    <div id="target-info">
      <div id="target1-box" class="target-box card">
        <h3>Target 1 Informations</h3>
        <div id="target1-data" class="target-data">No target detected</div>
      </div>
      <div id="target2-box" class="target-box card">
        <h3>Target 2 Informations</h3>
        <div id="target2-data" class="target-data">No target detected</div>
      </div>
      <div id="target3-box" class="target-box card">
        <h3>Target 3 Informations</h3>
        <div id="target3-data" class="target-data">No target detected</div>
      </div>
    </div>

    <!-- Warnings -->
    <div id="warnings" class="card">
      <h3>⚠️ Warnings</h3>
      <div id="warningList"></div>
    </div>
  </div>

  <script>
    const canvas = document.getElementById('radar');
    const ctx = canvas.getContext('2d');
    const statusEl = document.getElementById('status');

    let configuredRange = 5; // Vom Sensor eingestellte Reichweite
    const MAX_RANGE = 8; // Maximale Radar-Reichweite
    const PIXELS_PER_METER = 50;
    const CENTER_X = 400;
    const CENTER_Y = 30;

    const resetReasonMap = {
      1: 'POWERON_RESET',
      3: 'SW_RESET',
      4: 'OWDT_RESET',
      5: 'DEEPSLEEP_RESET',
      6: 'SDIO_RESET',
      7: 'TG0WDT_SYS_RESET',
      8: 'TG1WDT_SYS_RESET',
      9: 'RTCWDT_SYS_RESET',
      10: 'INTRUSION_RESET',
      11: 'TGWDT_CPU_RESET',
      12: 'SW_CPU_RESET',
      13: 'RTCWDT_CPU_RESET',
      14: 'EXT_CPU_RESET',
      15: 'RTCWDT_BROWN_OUT_RESET',
      16: 'RTCWDT_RTC_RESET'
    };

    function formatResetReason(code) {
      if (code === undefined || code === null) return '-';
      return resetReasonMap[code] || ('Code ' + code);
    }

    function sendCommand(cmd) {
      fetch('/api/cmd?cmd=' + cmd)
        .then(res => res.text())
        .then(data => {
          alert('Befehl gesendet: ' + cmd + '\nAntwort: ' + data);
        })
        .catch(err => {
          alert('Fehler beim Senden des Befehls: ' + err);
        });
    }

    // Polling statt WebSocket
    function fetchData() {
      fetch('/api/radar')
        .then(res => res.json())
        .then(data => {
          statusEl.className = 'connected';
          updateRadar(data);
        })
        .catch(err => {
          statusEl.className = 'disconnected';
          console.error('Fetch error:', err);
        });
    }

    function drawRadar() {
      ctx.fillStyle = '#1a1a2a';
      ctx.fillRect(0, 0, 800, 600);

      const maxRadius = MAX_RANGE * PIXELS_PER_METER;
      const configuredRadius = configuredRange * PIXELS_PER_METER;

      // Radial lines (nach Süden: Halbkreis nach unten, -90° bis +90°)
      ctx.strokeStyle = '#2a3a4a';
      ctx.lineWidth = 1;
      for(let angle = -90; angle <= 90; angle += 15) {
        const rad = (angle + 90) * Math.PI / 180;
        const x = CENTER_X + Math.cos(rad) * maxRadius;
        const y = CENTER_Y + Math.sin(rad) * maxRadius;
        ctx.beginPath();
        ctx.moveTo(CENTER_X, CENTER_Y);
        ctx.lineTo(x, y);
        ctx.stroke();
      }

      // Concentric arcs für alle 15 Meter
      ctx.strokeStyle = '#3a4a5a';
      ctx.lineWidth = 1;
      for(let m = 1; m <= MAX_RANGE; m++) {
        const r = m * PIXELS_PER_METER;
        ctx.beginPath();
        ctx.arc(CENTER_X, CENTER_Y, r, 0, Math.PI, false);
        ctx.stroke();
      }

      // Range labels für alle Meter
      ctx.fillStyle = '#6a7a8a';
      ctx.font = '12px monospace';
      ctx.textAlign = 'center';
      for(let m = 1; m <= MAX_RANGE; m++) {
        const y = CENTER_Y + m * PIXELS_PER_METER;
        ctx.fillText(m + 'm', CENTER_X + 25, y + 4);
      }

      // Gesamte Detection area (15m) - dezent
      const gradientMax = ctx.createRadialGradient(CENTER_X, CENTER_Y, 0, CENTER_X, CENTER_Y, maxRadius);
      gradientMax.addColorStop(0, 'rgba(70, 130, 180, 0.05)');
      gradientMax.addColorStop(1, 'rgba(70, 130, 180, 0.02)');
      ctx.fillStyle = gradientMax;
      ctx.beginPath();
      ctx.arc(CENTER_X, CENTER_Y, maxRadius, 0, Math.PI, false);
      ctx.lineTo(CENTER_X, CENTER_Y);
      ctx.closePath();
      ctx.fill();

      // Konfigurierte Reichweite - FARBIG hervorgehoben
      const gradientConfigured = ctx.createRadialGradient(CENTER_X, CENTER_Y, 0, CENTER_X, CENTER_Y, configuredRadius);
      gradientConfigured.addColorStop(0, 'rgba(52, 199, 89, 0.25)');
      gradientConfigured.addColorStop(1, 'rgba(52, 199, 89, 0.1)');
      ctx.fillStyle = gradientConfigured;
      ctx.beginPath();
      ctx.arc(CENTER_X, CENTER_Y, configuredRadius, 0, Math.PI, false);
      ctx.lineTo(CENTER_X, CENTER_Y);
      ctx.closePath();
      ctx.fill();

      // Markierung der konfigurierten Reichweite mit farbigem Bogen
      ctx.strokeStyle = '#34c759';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(CENTER_X, CENTER_Y, configuredRadius, 0, Math.PI, false);
      ctx.stroke();

      // Sensor position (oben/Norden)
      ctx.fillStyle = '#ff4444';
      ctx.beginPath();
      ctx.arc(CENTER_X, CENTER_Y, 8, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    function updateRadar(data) {
      drawRadar();

      // ESP Info aktualisieren
      document.getElementById('fwVersion').textContent = data.fwVersion || '-';
      document.getElementById('resetReason').textContent = formatResetReason(data.resetReason);
      const temp = data.temp_c;
      document.getElementById('temperature').textContent = (typeof temp === 'number' ? temp.toFixed(1) + ' °C' : '-');
      document.getElementById('radarTimeouts').textContent = (data.radarTimeouts !== undefined ? data.radarTimeouts : 0);
      document.getElementById('radarSerialRestarts').textContent = (data.radarSerialRestarts !== undefined ? data.radarSerialRestarts : 0);
      document.getElementById('ip').textContent = data.ip || '-';
      document.getElementById('targetCount').textContent = data.targetCount || 0;
      document.getElementById('maxRange').textContent = (data.range_m || 0).toFixed(1) + 'm';
      document.getElementById('uptime').textContent = (data.uptime_min || 0) + ' min';
      document.getElementById('rssi').textContent = (data.rssi || 0) + ' dBm';
      document.getElementById('heap').textContent = Math.round((data.heap_free || 0) / 1024) + ' KB';
      document.getElementById('holdMs').textContent = (data.holdMs || 500) + 'ms';

      configuredRange = data.range_m || 5;

      // Warnungen anzeigen
      const warningList = document.getElementById('warningList');
      if(data.warnings && data.warnings.length > 0) {
        warningList.innerHTML = data.warnings.map(w => '<div class="warning-item">⚠️ ' + w + '</div>').join('');
      } else {
        warningList.innerHTML = '<div style="color: #aaa; font-size: 13px;">Keine Warnungen</div>';
      }

      // Targets zeichnen und Boxen aktualisieren
      for(let i = 1; i <= 3; i++) {
        const t = data['target' + i];
        const targetData = document.getElementById('target' + i + '-data');

        if(!t || !t.presence) {
          targetData.innerHTML = '<div style="color: rgba(245, 245, 247, 0.4); text-align: center; padding: 20px;">No target detected</div>';
          continue;
        }

        // Target Box aktualisieren
        targetData.innerHTML = `
          <div><strong>Distance:</strong> ${(t.distance/1000).toFixed(2)}m</div>
          <div><strong>Angle:</strong> ${t.angleDeg.toFixed(0)}°</div>
          <div><strong>X:</strong> ${(t.x/1000).toFixed(2)}m</div>
          <div><strong>Y:</strong> ${(t.y/1000).toFixed(2)}m</div>
          <div><strong>Speed:</strong> ${t.speed}</div>
        `;

        const xMeters = t.x / 1000.0;
        const yMeters = t.y / 1000.0;

        // 180° gedreht: Y jetzt nach unten positiv
        const screenX = CENTER_X + xMeters * PIXELS_PER_METER;
        const screenY = CENTER_Y + yMeters * PIXELS_PER_METER;

        // Target glow
        const gradient = ctx.createRadialGradient(screenX, screenY, 0, screenX, screenY, 15);
        gradient.addColorStop(0, 'rgba(255, 100, 100, 0.8)');
        gradient.addColorStop(1, 'rgba(255, 100, 100, 0)');
        ctx.fillStyle = gradient;
        ctx.beginPath();
        ctx.arc(screenX, screenY, 15, 0, Math.PI * 2);
        ctx.fill();

        // Target core
        ctx.fillStyle = '#ff6464';
        ctx.beginPath();
        ctx.arc(screenX, screenY, 5, 0, Math.PI * 2);
        ctx.fill();

        // Label
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 12px monospace';
        ctx.textAlign = 'center';
        ctx.strokeStyle = '#000';
        ctx.lineWidth = 3;
        ctx.strokeText('T' + i, screenX, screenY - 20);
        ctx.fillText('T' + i, screenX, screenY - 20);
      }
    }

    // Poll every 1000ms to reduce socket churn
    setInterval(fetchData, 1000);
    fetchData();
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  webServer.send_P(200, "text/html", INDEX_HTML);
}

void handleRadarAPI() {
  StaticJsonDocument<JSON_BUFFER_SIZE> doc;

  doc["targetCount"] = 0;
  for (auto &t: smoothed) if (t.presence) doc["targetCount"] = doc["targetCount"].as<int>() + 1;

  doc["fwVersion"] = FW_VERSION;
  doc["resetReason"] = esp_reset_reason();
  doc["temp_c"] = temperatureRead();
  doc["radarTimeouts"] = radarTimeoutCount;
  doc["radarSerialRestarts"] = radarSerialRestartCount;
  doc["range_m"] = g_maxRangeMeters;
  doc["uptime_min"] = millis() / 60000;
  doc["rssi"] = WiFi.RSSI();
  doc["ip"] = WiFi.localIP().toString();
  doc["heap_free"] = ESP.getFreeHeap();
  doc["holdMs"] = g_holdIntervalMs;

  // Warnungen hinzufügen
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

  for (int i = 0; i < 3; i++) {
    char key[12];
    snprintf(key, sizeof(key), "target%d", i + 1);
    auto t = doc.createNestedObject(key);

    if (!smoothed[i].presence) {
      t["presence"] = false;
    } else {
      t["presence"] = true;
      t["x"] = round(smoothed[i].x);
      t["y"] = round(smoothed[i].y);
      t["speed"] = round(smoothed[i].speed);
      t["distance"] = round(smoothed[i].distanceXY);
      t["angleDeg"] = round(smoothed[i].angleDeg);
    }
  }

  char buffer[JSON_BUFFER_SIZE];
  serializeJson(doc, buffer);

  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", buffer);
}

void handleCommand() {
  if (!webServer.hasArg("cmd")) {
    webServer.send(400, "text/plain", "ERROR: Kein Befehl angegeben");
    return;
  }

  String cmd = webServer.arg("cmd");
  processMqttCommand(cmd);

  webServer.send(200, "text/plain", "OK: Befehl '" + cmd + "' wurde ausgeführt");
}

void setupWebServer() {
  webServer.on("/", handleRoot);
  webServer.on("/api/radar", handleRadarAPI);
  webServer.on("/api/cmd", handleCommand);

  webServer.begin();
  Serial.println("WebServer started on port 80");
}

void handleWebServer() {
  webServer.handleClient();
}

void sendRadarData() {
  // Not needed with polling approach
}
