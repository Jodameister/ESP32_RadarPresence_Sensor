// File: WebServerHandler.cpp
// Uses standard ESP32 WebServer - no external libraries needed

#include "WebServerHandler.h"
#include "Config.h"
#include "RadarHandler.h"
#include "MQTTHandler.h"
#include <ArduinoJson.h>
#include <esp_system.h>

WebServer webServer(80);
static bool serverConfigured = false;
static bool serverRunning = false;

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

// Track SSE clients
struct SSEClient {
  WiFiClient client;
  unsigned long lastPing;
  bool active;
};

SSEClient sseClients[1];
const int MAX_SSE_CLIENTS = 1;
unsigned long lastSseBroadcast = 0;

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

    :root {
      --body-bg: #000000;
      --body-text: #f5f5f7;
      --muted-text: rgba(245, 245, 247, 0.7);
      --card-bg: rgba(29, 29, 31, 0.72);
      --card-border: rgba(255, 255, 255, 0.1);
      --card-border-hover: rgba(255, 255, 255, 0.15);
      --card-text: #f5f5f7;
      --card-shadow: none;
      --card-shadow-hover: 0 8px 32px rgba(0, 0, 0, 0.3);
      --canvas-bg: rgba(0, 0, 0, 0.4);
      --canvas-fill: #1a1a2a;
      --canvas-border: rgba(255, 255, 255, 0.05);
      --status-connected-bg: rgba(52, 199, 89, 0.15);
      --status-connected-border: rgba(52, 199, 89, 0.3);
      --status-connected-text: #34c759;
      --status-disconnected-bg: rgba(255, 69, 58, 0.15);
      --status-disconnected-border: rgba(255, 69, 58, 0.3);
      --status-disconnected-text: #ff453a;
      --btn-bg: rgba(99, 102, 241, 0.15);
      --btn-hover-bg: rgba(99, 102, 241, 0.25);
      --btn-text: #f5f5f7;
      --btn-border: rgba(99, 102, 241, 0.35);
      --btn-ripple: rgba(255, 255, 255, 0.1);
      --esp-info-gradient: linear-gradient(135deg, rgba(0, 122, 255, 0.1), rgba(52, 199, 89, 0.1));
      --radar-settings-gradient: linear-gradient(135deg, rgba(255, 204, 0, 0.1), rgba(255, 149, 0, 0.1));
      --warnings-gradient: linear-gradient(135deg, rgba(0, 122, 255, 0.1), rgba(10, 132, 255, 0.1));
      --button-shadow: 0 10px 20px rgba(0, 0, 0, 0.25);
      --canvas-grid-color: #2a3a4a;
      --canvas-arc-color: #3a4a5a;
      --canvas-label-color: #6a7a8a;
      --divider-color: rgba(255, 255, 255, 0.05);
      --accent-color: #0a84ff;
      --target-box-gradient: linear-gradient(135deg, rgba(255, 159, 10, 0.1), rgba(255, 204, 0, 0.1));
      --target-strong-color: rgba(245, 245, 247, 0.6);
    }

    body {
      font-family: -apple-system, BlinkMacSystemFont, "SF Pro Display", "SF Pro Text", "Segoe UI", Arial, sans-serif;
      background: var(--body-bg);
      color: var(--body-text);
      margin: 0;
      padding: 12px 20px;
      min-height: 100vh;
      -webkit-font-smoothing: antialiased;
      -moz-osx-font-smoothing: grayscale;
    }

    body.dark-mode {
      --body-bg: #000000;
      --body-text: #f5f5f7;
      --muted-text: rgba(245, 245, 247, 0.7);
      --card-bg: rgba(29, 29, 31, 0.72);
      --card-border: rgba(255, 255, 255, 0.1);
      --card-border-hover: rgba(255, 255, 255, 0.15);
      --card-text: #f5f5f7;
      --card-shadow: none;
      --card-shadow-hover: 0 8px 32px rgba(0, 0, 0, 0.3);
      --canvas-bg: rgba(0, 0, 0, 0.4);
      --canvas-fill: #1a1a2a;
      --canvas-border: rgba(255, 255, 255, 0.05);
      --status-connected-bg: rgba(52, 199, 89, 0.15);
      --status-connected-border: rgba(52, 199, 89, 0.3);
      --status-connected-text: #34c759;
      --status-disconnected-bg: rgba(255, 69, 58, 0.15);
      --status-disconnected-border: rgba(255, 69, 58, 0.3);
      --status-disconnected-text: #ff453a;
      --btn-bg: rgba(99, 102, 241, 0.15);
      --btn-hover-bg: rgba(99, 102, 241, 0.25);
      --btn-text: #f5f5f7;
      --btn-border: rgba(99, 102, 241, 0.35);
      --btn-ripple: rgba(255, 255, 255, 0.1);
      --esp-info-gradient: linear-gradient(135deg, rgba(0, 122, 255, 0.1), rgba(52, 199, 89, 0.1));
      --radar-settings-gradient: linear-gradient(135deg, rgba(255, 204, 0, 0.1), rgba(255, 149, 0, 0.1));
      --warnings-gradient: linear-gradient(135deg, rgba(0, 122, 255, 0.1), rgba(10, 132, 255, 0.1));
      --button-shadow: 0 10px 20px rgba(0, 0, 0, 0.25);
      --canvas-grid-color: #2a3a4a;
      --canvas-arc-color: #3a4a5a;
      --canvas-label-color: #6a7a8a;
      --divider-color: rgba(255, 255, 255, 0.05);
      --accent-color: #0a84ff;
      --target-box-gradient: linear-gradient(135deg, rgba(255, 159, 10, 0.1), rgba(255, 204, 0, 0.1));
      --target-strong-color: rgba(245, 245, 247, 0.6);
    }

    body.light-mode {
      --body-bg: #f5f5f7;
      --body-text: #1c1c1e;
      --muted-text: rgba(60, 60, 67, 0.6);
      --card-bg: rgba(255, 255, 255, 0.92);
      --card-border: rgba(0, 0, 0, 0.08);
      --card-border-hover: rgba(0, 0, 0, 0.18);
      --card-text: #1c1c1e;
      --card-shadow: 0 4px 18px rgba(0, 0, 0, 0.08);
      --card-shadow-hover: 0 16px 36px rgba(0, 0, 0, 0.12);
      --canvas-bg: rgba(255, 255, 255, 0.65);
      --canvas-fill: #ffffff;
      --canvas-border: rgba(0, 0, 0, 0.08);
      --status-connected-bg: rgba(76, 217, 100, 0.2);
      --status-connected-border: rgba(76, 217, 100, 0.45);
      --status-connected-text: #2c7c33;
      --status-disconnected-bg: rgba(255, 69, 58, 0.18);
      --status-disconnected-border: rgba(255, 69, 58, 0.35);
      --status-disconnected-text: #b3261e;
      --btn-bg: rgba(0, 122, 255, 0.12);
      --btn-hover-bg: rgba(0, 122, 255, 0.2);
      --btn-text: #0b1a33;
      --btn-border: rgba(0, 122, 255, 0.25);
      --btn-ripple: rgba(0, 0, 0, 0.12);
      --esp-info-gradient: linear-gradient(135deg, rgba(0, 122, 255, 0.18), rgba(52, 199, 89, 0.18));
      --radar-settings-gradient: linear-gradient(135deg, rgba(255, 204, 0, 0.22), rgba(255, 149, 0, 0.18));
      --warnings-gradient: linear-gradient(135deg, rgba(0, 122, 255, 0.2), rgba(10, 132, 255, 0.16));
      --button-shadow: 0 10px 25px rgba(15, 23, 42, 0.15);
      --canvas-grid-color: rgba(60, 60, 67, 0.18);
      --canvas-arc-color: rgba(60, 60, 67, 0.25);
      --canvas-label-color: rgba(60, 60, 67, 0.45);
      --divider-color: rgba(0, 0, 0, 0.08);
      --accent-color: #0a84ff;
      --target-box-gradient: linear-gradient(135deg, rgba(255, 183, 77, 0.18), rgba(255, 214, 102, 0.18));
      --target-strong-color: rgba(60, 60, 67, 0.7);
    }

    #theme-switcher {
      display: flex;
      justify-content: flex-end;
      align-items: center;
      max-width: 1600px;
      margin: 0 auto 16px;
    }

    .theme-toggle-btn {
      border: 1px solid var(--btn-border);
      background: var(--btn-bg);
      color: var(--btn-text);
      padding: 10px 20px;
      border-radius: 999px;
      font-size: 14px;
      font-weight: 600;
      cursor: pointer;
      transition: background 0.2s ease, transform 0.2s ease, box-shadow 0.2s ease;
      box-shadow: var(--button-shadow);
      backdrop-filter: blur(12px);
    }

    .theme-toggle-btn:hover {
      background: var(--btn-hover-bg);
      transform: translateY(-1px);
    }

    .toggle-btn {
      border: 1px solid var(--btn-border);
      background: var(--btn-bg);
      color: var(--btn-text);
      padding: 6px 14px;
      border-radius: 999px;
      font-size: 13px;
      font-weight: 600;
      cursor: pointer;
      transition: background 0.2s ease, transform 0.2s ease, box-shadow 0.2s ease;
      box-shadow: var(--button-shadow);
      backdrop-filter: blur(12px);
    }

    .toggle-btn:hover {
      background: var(--btn-hover-bg);
      transform: translateY(-1px);
    }

    .toggle-btn.active {
      background: var(--btn-hover-bg);
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
      background: var(--status-connected-bg);
      border: 1px solid var(--status-connected-border);
      color: var(--status-connected-text);
    }
    #status.disconnected {
      background: var(--status-disconnected-bg);
      border: 1px solid var(--status-disconnected-border);
      color: var(--status-disconnected-text);
    }

    /* Dashboard Grid */
    #dashboard {
      display: grid;
      grid-template-columns: 260px 1fr 1fr 1fr;
      grid-template-rows: auto auto auto auto;
      grid-template-areas:
        "esp radar radar radar"
        "buttons radar radar radar"
        "settings target1 target2 target3"
        "warnings warnings warnings warnings";
      gap: 16px;
      max-width: 1600px;
      margin: 0 auto;
      align-items: stretch;
    }

    /* Card Base Style - Apple Glassmorphism */
    .card {
      background: var(--card-bg);
      backdrop-filter: saturate(180%) blur(20px);
      border-radius: 18px;
      border: 1px solid var(--card-border);
      padding: 24px;
      transition: all 0.3s ease;
      box-shadow: var(--card-shadow);
      color: var(--card-text);
    }
    .card:hover {
      border-color: var(--card-border-hover);
      box-shadow: var(--card-shadow-hover);
    }
    .card h3 {
      font-size: 17px;
      font-weight: 600;
      margin-bottom: 16px;
      color: var(--card-text);
      letter-spacing: -0.3px;
    }

    /* ESP Info Box */
    #esp-info {
      grid-area: esp;
      background: var(--esp-info-gradient);
    }

    /* Radar Canvas */
    canvas {
      grid-area: radar;
      background: var(--canvas-bg);
      border-radius: 12px;
      width: 100%;
      height: 100%;
      border: 1px solid var(--canvas-border);
      min-height: 420px;
    }

    /* Control Buttons */
    #control-buttons {
      grid-area: buttons;
      display: flex;
      flex-direction: column;
      gap: 12px;
    }

    /* Radar Settings */
    #radar-settings {
      grid-area: settings;
      background: var(--radar-settings-gradient);
    }

    /* Target Info */
    #target1-box { grid-area: target1; }
    #target2-box { grid-area: target2; }
    #target3-box { grid-area: target3; }

    /* Warnings */
    #warnings {
      grid-area: warnings;
      background: var(--warnings-gradient);
    }
    /* Apple Buttons */
    .btn {
      padding: 14px 20px;
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
      border: 1px solid var(--btn-border);
      background: var(--btn-bg);
      color: var(--btn-text);
      box-shadow: var(--button-shadow);
      backdrop-filter: blur(12px);
    }
    .btn::before {
      content: '';
      position: absolute;
      top: 50%;
      left: 50%;
      width: 0;
      height: 0;
      border-radius: 50%;
      background: var(--btn-ripple);
      transform: translate(-50%, -50%);
      transition: width 0.6s, height 0.6s;
    }
    .btn:hover::before {
      width: 300px;
      height: 300px;
    }
    .btn:hover {
      background: var(--btn-hover-bg);
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
      border-bottom: 1px solid var(--divider-color);
    }
    .info-item:last-child {
      border-bottom: none;
    }
    .info-label {
      font-weight: 500;
      color: var(--muted-text);
    }
    .info-value {
      color: var(--accent-color);
      font-weight: 600;
    }

    /* Target Boxes */
    .target-box {
      background: var(--target-box-gradient);
      height: 100%;
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
      border-bottom: 1px solid var(--divider-color);
    }
    .target-data div:last-child {
      border-bottom: none;
    }
    .target-data strong {
      color: var(--target-strong-color);
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
      #theme-switcher {
        justify-content: center;
        margin-bottom: 12px;
      }
      #dashboard {
        grid-template-columns: 1fr;
        grid-template-areas:
          "radar"
          "esp"
          "target1"
          "target2"
          "target3"
          "settings"
          "buttons"
          "warnings";
      }
      canvas {
        min-height: 300px;
      }
      #control-buttons {
        flex-direction: column;
      }
    }
  </style>
</head>
<body class="dark-mode">
  <div id="theme-switcher">
    <button id="themeToggle" class="theme-toggle-btn" type="button">☀️ Light Mode</button>
  </div>
  <h1>🎯 Radar Live Monitor</h1>
  <div id="status" class="connected">Verbunden</div>

  <div id="dashboard">
    <!-- ESP Infos -->
    <div id="esp-info" class="card">
      <h3>ESP Infos</h3>
      <div class="info-item"><span class="info-label">Firmware:</span> <span id="fwVersion" class="info-value">-</span></div>
      <div class="info-item"><span class="info-label">Reset Reason:</span> <span id="resetReason" class="info-value">-</span></div>
      <div class="info-item"><span class="info-label">Temperatur:</span> <span id="temperature" class="info-value">-</span></div>
      <div class="info-item"><span class="info-label">Radar Restarts:</span> <span id="radarSerialRestarts" class="info-value">0</span></div>
      <div class="info-item"><span class="info-label">IP:</span> <span id="ip" class="info-value">-</span></div>
      <div class="info-item"><span class="info-label">Uptime:</span> <span id="uptime" class="info-value">000:00</span></div>
      <div class="info-item"><span class="info-label">RSSI:</span> <span id="rssi" class="info-value">0 dBm</span></div>
      <div class="info-item"><span class="info-label">Heap:</span> <span id="heap" class="info-value">0 KB</span></div>
      <div class="info-item"><span class="info-label">Targets:</span> <span id="targetCount" class="info-value">0</span></div>
      <div class="info-item"><span class="info-label">X-Achse:</span> <button id="invertToggle" class="toggle-btn" type="button">Normal</button></div>
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
    <div id="target1-box" class="target-box card">
      <h3>Target 1</h3>
      <div id="target1-data" class="target-data">No target detected</div>
    </div>
    <div id="target2-box" class="target-box card">
      <h3>Target 2</h3>
      <div id="target2-data" class="target-data">No target detected</div>
    </div>
    <div id="target3-box" class="target-box card">
      <h3>Target 3</h3>
      <div id="target3-data" class="target-data">No target detected</div>
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
    const themeToggle = document.getElementById('themeToggle');
    const invertToggle = document.getElementById('invertToggle');
    const THEME_STORAGE_KEY = 'rp-theme';
    const INVERT_COOKIE_KEY = 'rp-invert-x';
    const prefersDarkQuery = window.matchMedia ? window.matchMedia('(prefers-color-scheme: dark)') : null;

    function varFallback(value, fallback) {
      if (typeof value !== 'string') return fallback;
      const trimmed = value.trim();
      return trimmed.length ? trimmed : fallback;
    }

    function readCookie(name) {
      const match = document.cookie.match(new RegExp('(?:^|; )' + name + '=([^;]*)'));
      return match ? decodeURIComponent(match[1]) : null;
    }

    function writeCookie(name, value, days) {
      const maxAge = (days || 365) * 24 * 60 * 60;
      document.cookie = name + '=' + encodeURIComponent(value) + '; path=/; max-age=' + maxAge + '; SameSite=Lax';
    }

    let configuredRange = 5; // Vom Sensor eingestellte Reichweite
    const MAX_RANGE = 8; // Maximale Radar-Reichweite
    const PIXELS_PER_METER = 50;
    const CENTER_X = 400;
    const CENTER_Y = 30;
    let fallbackTimer = null;
    let eventSource = null;
    let invertXAxis = false;
    let lastRadarPayload = null;

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
      if (typeof code === 'string') return code;
      return resetReasonMap[code] || ('code ' + code);
    }

    function applyTheme(mode) {
      const isLight = mode === 'light';
      document.body.classList.toggle('light-mode', isLight);
      document.body.classList.toggle('dark-mode', !isLight);
      if (themeToggle) {
        themeToggle.textContent = isLight ? '🌙 Dark Mode' : '☀️ Light Mode';
      }
    }

    function updateInvertToggleLabel() {
      if (!invertToggle) return;
      invertToggle.textContent = invertXAxis ? 'Invertiert' : 'Normal';
      invertToggle.classList.toggle('active', invertXAxis);
      invertToggle.setAttribute('aria-pressed', invertXAxis ? 'true' : 'false');
    }

    function initTheme() {
      let storedMode = null;
      try {
        storedMode = localStorage.getItem(THEME_STORAGE_KEY);
      } catch (err) {
        storedMode = null;
      }

      let initialMode = storedMode;
      if (initialMode !== 'light' && initialMode !== 'dark') {
        const prefersDark = prefersDarkQuery ? prefersDarkQuery.matches : true;
        initialMode = prefersDark ? 'dark' : 'light';
      }

      applyTheme(initialMode);

      if (themeToggle) {
        themeToggle.addEventListener('click', () => {
          const newMode = document.body.classList.contains('light-mode') ? 'dark' : 'light';
          applyTheme(newMode);
          try {
            localStorage.setItem(THEME_STORAGE_KEY, newMode);
          } catch (err) {
            /* ignore storage errors */
          }
        });
      }

      const handleSystemThemeChange = (event) => {
        let stored = null;
        try {
          stored = localStorage.getItem(THEME_STORAGE_KEY);
        } catch (err) {
          stored = null;
        }
        if (stored === 'light' || stored === 'dark') {
          return; // user preference overrides system
        }
        applyTheme(event.matches ? 'dark' : 'light');
      };

      if (prefersDarkQuery) {
        if (prefersDarkQuery.addEventListener) {
          prefersDarkQuery.addEventListener('change', handleSystemThemeChange);
        } else if (prefersDarkQuery.addListener) {
          prefersDarkQuery.addListener(handleSystemThemeChange);
        }
      }
    }

    function initInvertToggle() {
      const stored = readCookie(INVERT_COOKIE_KEY);
      invertXAxis = stored === '1';
      updateInvertToggleLabel();

      if (invertToggle) {
        invertToggle.addEventListener('click', () => {
          invertXAxis = !invertXAxis;
          updateInvertToggleLabel();
          writeCookie(INVERT_COOKIE_KEY, invertXAxis ? '1' : '0', 365);
          if (lastRadarPayload) {
            updateRadar(lastRadarPayload);
          } else {
            drawRadar();
          }
        });
      }
    }

    function formatUptimeLabel(minutes, formatted) {
      if (formatted && typeof formatted === 'string') {
        return formatted;
      }
      const totalMinutes = Number.isFinite(minutes) ? Math.max(0, Math.floor(minutes)) : 0;
      const hours = Math.min(999, Math.floor(totalMinutes / 60));
      const mins = totalMinutes % 60;
      return hours.toString().padStart(3, '0') + ':' + mins.toString().padStart(2, '0');
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

    function startPollingFallback() {
      if (fallbackTimer) return;
      fetchData();
      fallbackTimer = setInterval(fetchData, 1000);
    }

    function setupRealtime() {
      if (fallbackTimer) {
        clearInterval(fallbackTimer);
        fallbackTimer = null;
      }
      if (!window.EventSource) {
        startPollingFallback();
        return;
      }

      if (eventSource) {
        eventSource.close();
      }
      eventSource = new EventSource('/events');
      eventSource.onopen = () => {
        statusEl.className = 'connected';
      };
      eventSource.onmessage = (event) => {
        try {
          const payload = JSON.parse(event.data);
          statusEl.className = 'connected';
          updateRadar(payload);
        } catch (e) {
          console.error('SSE parse error', e);
        }
      };
      eventSource.onerror = (err) => {
        console.warn('SSE error, fallback to polling', err);
        statusEl.className = 'disconnected';
        if (eventSource) {
          eventSource.close();
          eventSource = null;
        }
        startPollingFallback();
      };
    }

    function drawRadar() {
      const styles = getComputedStyle(document.body);
      const gridColor = varFallback(styles.getPropertyValue('--canvas-grid-color'), '#2a3a4a');
      const arcColor = varFallback(styles.getPropertyValue('--canvas-arc-color'), '#3a4a5a');
      const labelColor = varFallback(styles.getPropertyValue('--canvas-label-color'), '#6a7a8a');

      ctx.fillStyle = varFallback(styles.getPropertyValue('--canvas-fill'), '#1a1a2a');
      ctx.fillRect(0, 0, 800, 600);

      const maxRadius = MAX_RANGE * PIXELS_PER_METER;
      const configuredRadius = configuredRange * PIXELS_PER_METER;

      // Radial lines (nach Süden: Halbkreis nach unten, -90° bis +90°)
      ctx.strokeStyle = gridColor;
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
      ctx.strokeStyle = arcColor;
      ctx.lineWidth = 1;
      for(let m = 1; m <= MAX_RANGE; m++) {
        const r = m * PIXELS_PER_METER;
        ctx.beginPath();
        ctx.arc(CENTER_X, CENTER_Y, r, 0, Math.PI, false);
        ctx.stroke();
      }

      // Range labels für alle Meter
      ctx.fillStyle = labelColor;
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
      if (typeof data.range_m === 'number') {
        configuredRange = data.range_m;
      }
      lastRadarPayload = data;
      drawRadar();

      // ESP Info aktualisieren
      document.getElementById('fwVersion').textContent = data.fwVersion || '-';
      document.getElementById('resetReason').textContent = formatResetReason(data.resetReason);
      const temp = data.temp_c;
      document.getElementById('temperature').textContent = (typeof temp === 'number' ? temp.toFixed(1) + ' °C' : '-');
      document.getElementById('radarSerialRestarts').textContent = (data.radarSerialRestarts !== undefined ? data.radarSerialRestarts : 0);
      document.getElementById('ip').textContent = data.ip || '-';
      document.getElementById('targetCount').textContent = data.targetCount || 0;
      document.getElementById('maxRange').textContent = (configuredRange || 0).toFixed(1) + 'm';
      document.getElementById('uptime').textContent = formatUptimeLabel(data.uptime_min, data.uptime);
      document.getElementById('rssi').textContent = (data.rssi || 0) + ' dBm';
      document.getElementById('heap').textContent = Math.round((data.heap_free || 0) / 1024) + ' KB';
      document.getElementById('holdMs').textContent = (data.holdMs || 500) + 'ms';

      configuredRange = data.range_m || 5;

      // Warnungen anzeigen
      const warningList = document.getElementById('warningList');
      if(data.warnings && data.warnings.length > 0) {
        warningList.innerHTML = data.warnings.map(w => '<div class="warning-item">⚠️ ' + w + '</div>').join('');
      } else {
        warningList.innerHTML = '<div style="color: var(--muted-text); font-size: 13px;">Keine Warnungen</div>';
      }

      // Targets zeichnen und Boxen aktualisieren
      for(let i = 1; i <= 3; i++) {
        const t = data['target' + i];
        const targetData = document.getElementById('target' + i + '-data');

        if(!t || !t.presence) {
          targetData.innerHTML = '<div style="color: var(--muted-text); text-align: center; padding: 20px;">No target detected</div>';
          continue;
        }

        const rawX = typeof t.x === 'number' ? t.x : 0;
        const rawY = typeof t.y === 'number' ? t.y : 0;
        const rawAngle = typeof t.angleDeg === 'number' ? t.angleDeg : 0;
        const xMeters = (invertXAxis ? -rawX : rawX) / 1000.0;
        const yMeters = rawY / 1000.0;
        const displayAngle = invertXAxis ? -rawAngle : rawAngle;
        const displayDistance = typeof t.distance === 'number' ? t.distance / 1000.0 : (typeof t.distRaw === 'number' ? t.distRaw / 1000.0 : 0);
        const displaySpeed = typeof t.speed === 'number' ? t.speed : 0;

        // Target Box aktualisieren
        targetData.innerHTML = `
          <div><strong>Distance:</strong> ${displayDistance.toFixed(2)}m</div>
          <div><strong>Angle:</strong> ${displayAngle.toFixed(0)}°</div>
          <div><strong>X:</strong> ${xMeters.toFixed(2)}m</div>
          <div><strong>Y:</strong> ${yMeters.toFixed(2)}m</div>
          <div><strong>Speed:</strong> ${displaySpeed}</div>
        `;

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

    initTheme();
    initInvertToggle();
    drawRadar();
    setupRealtime();
  </script>
</body>
</html>
)rawliteral";

static size_t buildRadarJson(char* buffer, size_t bufsize) {
  StaticJsonDocument<JSON_BUFFER_SIZE> doc;

  doc["targetCount"] = 0;
  for (auto &t: smoothed) if (t.presence) doc["targetCount"] = doc["targetCount"].as<int>() + 1;

  doc["fwVersion"] = FW_VERSION;
  doc["resetReason"] = resetReasonToString(esp_reset_reason());
  doc["temp_c"] = temperatureRead();
  doc["radarSerialRestarts"] = radarSerialRestartCount;
  doc["range_m"] = g_maxRangeMeters;
  doc["uptime_min"] = millis() / 60000;
  char uptimeStr[8];
  formatUptime(uptimeStr, sizeof(uptimeStr));
  doc["uptime"] = uptimeStr;
  doc["rssi"] = WiFi.RSSI();
  doc["ip"] = WiFi.localIP().toString();
  doc["heap_free"] = ESP.getFreeHeap();
  doc["holdMs"] = g_holdIntervalMs;

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

  return serializeJson(doc, buffer, bufsize);
}

void handleSSE() {
  SSEClient& slot = sseClients[0];
  if (slot.active) {
    slot.client.stop();
    slot.active = false;
  }

  WiFiClient client = webServer.client();
  if (!client.connected()) {
    return;
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/event-stream");
  client.println("Cache-Control: no-cache");
  client.println("Connection: keep-alive");
  client.println();
  client.flush();

  slot.client = client;
  slot.lastPing = millis();
  slot.active = true;
  lastSseBroadcast = 0;

  char buffer[JSON_BUFFER_SIZE];
  size_t len = buildRadarJson(buffer, sizeof(buffer));
  slot.client.print("data: ");
  slot.client.write(buffer, len);
  slot.client.print("\n\n");
  slot.client.flush();
  lastSseBroadcast = slot.lastPing;
}

void broadcastRadarSSE() {
  SSEClient& slot = sseClients[0];
  if (!slot.active) return;
  if (!slot.client.connected()) {
    slot.client.stop();
    slot.active = false;
    return;
  }

  unsigned long now = millis();
  if (now - lastSseBroadcast < 500) return;

  char buffer[JSON_BUFFER_SIZE];
  size_t len = buildRadarJson(buffer, sizeof(buffer));

  slot.client.print("data: ");
  slot.client.write(buffer, len);
  slot.client.print("\n\n");
  slot.client.flush();

  slot.lastPing = now;
  lastSseBroadcast = now;
}

void handleRoot() {
  webServer.send_P(200, "text/html", INDEX_HTML);
}

void handleRadarAPI() {
  char buffer[JSON_BUFFER_SIZE];
  buildRadarJson(buffer, sizeof(buffer));
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
  if (!serverConfigured) {
    webServer.on("/", handleRoot);
    webServer.on("/api/radar", handleRadarAPI);
    webServer.on("/events", handleSSE);
    webServer.on("/api/cmd", handleCommand);
    serverConfigured = true;
  }

  if (!serverRunning) {
    webServer.begin();
    serverRunning = true;
    Serial.println("WebServer started on port 80");
  }
}

void stopWebServer() {
  if (!serverRunning) return;
  for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
    if (sseClients[i].client.connected()) {
      sseClients[i].client.stop();
    }
    sseClients[i].active = false;
  }
  webServer.stop();
  serverRunning = false;
  Serial.println("WebServer stopped");
}

void handleWebServer() {
  if (!serverRunning) return;
  webServer.handleClient();
  broadcastRadarSSE();
}

bool isWebServerRunning() {
  return serverRunning;
}

void sendRadarData() {
  // Not needed with polling approach
}
