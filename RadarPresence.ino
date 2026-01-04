//************************************************************
// ESP32 RadarPresence – Main sketch
//************************************************************

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <SHA2Builder.h>
#include <PBKDF2_HMACBuilder.h>

#include "Config.h"
#include "RadarHandler.h"
#include "MQTTHandler.h"
#include "OTAHandler.h"
#include "WebServerHandler.h"

// WiFiManager parameter handling
struct ParamInfo {
  const char* id;
  const char* label;
  String& val;
  int len;
  const char* html;
};

ParamInfo paramInfos[] = {
  {"mqtt_server","MQTT Server", g_mqttServer, 16, "type='text' maxlength='15'"},
  {"mqtt_port",  "MQTT Port",   g_mqttPort,   6,  "type='text' maxlength='5'"},
  {"mqtt_topic", "MQTT Topic",  g_mqttTopic,  64, "type='text' maxlength='63'"},
  {"radar_rx",   "Radar Rx Pin",g_radarRxPin, 3,  "type='text' maxlength='2'"},
  {"radar_tx",   "Radar Tx Pin",g_radarTxPin, 3,  "type='text' maxlength='2'"},
  {"host",       "Hostname",    g_host,       20, "type='text' maxlength='20'"},
  {"otaPass",    "OTA Password",g_otaPass,    10, "type='text' maxlength='10'"}
};

WiFiManagerParameter paramObjects[sizeof(paramInfos)/sizeof(paramInfos[0])];

void configureWiFiManager(WiFiManager& wm);
void setupWiFiManagerParams(WiFiManager& wm);
void handleMqttCommands();
void maintainWiFi();
bool syncConfigFromWiFiManager();
const char* wifiDisconnectReason(uint8_t reason);
const char* wifiAuthModeName(wifi_auth_mode_t auth);
void logApInfo(const char* prefix);
unsigned long lastWiFiConnectEvent = 0;

const char* wifiDisconnectReason(uint8_t reason) {
  switch (reason) {
    case WIFI_REASON_UNSPECIFIED: return "UNSPECIFIED";
    case WIFI_REASON_AUTH_EXPIRE: return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE: return "AUTH_LEAVE";
    case WIFI_REASON_ASSOC_EXPIRE: return "ASSOC_EXPIRE";
    case WIFI_REASON_ASSOC_TOOMANY: return "ASSOC_TOOMANY";
    case WIFI_REASON_NOT_AUTHED: return "NOT_AUTHED";
    case WIFI_REASON_NOT_ASSOCED: return "NOT_ASSOCED";
    case WIFI_REASON_ASSOC_LEAVE: return "ASSOC_LEAVE";
    case WIFI_REASON_ASSOC_NOT_AUTHED: return "ASSOC_NOT_AUTHED";
    case WIFI_REASON_DISASSOC_PWRCAP_BAD: return "DISASSOC_PWRCAP_BAD";
    case WIFI_REASON_DISASSOC_SUPCHAN_BAD: return "DISASSOC_SUPCHAN_BAD";
    case WIFI_REASON_IE_INVALID: return "IE_INVALID";
    case WIFI_REASON_MIC_FAILURE: return "MIC_FAILURE";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "4WAY_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT: return "GROUP_KEY_UPDATE_TIMEOUT";
    case WIFI_REASON_IE_IN_4WAY_DIFFERS: return "IE_IN_4WAY_DIFFERS";
    case WIFI_REASON_GROUP_CIPHER_INVALID: return "GROUP_CIPHER_INVALID";
    case WIFI_REASON_PAIRWISE_CIPHER_INVALID: return "PAIRWISE_CIPHER_INVALID";
    case WIFI_REASON_AKMP_INVALID: return "AKMP_INVALID";
    case WIFI_REASON_UNSUPP_RSN_IE_VERSION: return "UNSUPP_RSN_IE_VERSION";
    case WIFI_REASON_INVALID_RSN_IE_CAP: return "INVALID_RSN_IE_CAP";
    case WIFI_REASON_802_1X_AUTH_FAILED: return "802_1X_AUTH_FAILED";
    case WIFI_REASON_CIPHER_SUITE_REJECTED: return "CIPHER_SUITE_REJECTED";
    case WIFI_REASON_BEACON_TIMEOUT: return "BEACON_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND: return "NO_AP_FOUND";
    case WIFI_REASON_AUTH_FAIL: return "AUTH_FAIL";
    case WIFI_REASON_ASSOC_FAIL: return "ASSOC_FAIL";
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "HANDSHAKE_TIMEOUT";
    default: return "UNKNOWN";
  }
}

const char* wifiAuthModeName(wifi_auth_mode_t auth) {
  switch (auth) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2_PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
    case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2_WPA3_PSK";
    case WIFI_AUTH_WAPI_PSK: return "WAPI_PSK";
    default: return "UNKNOWN";
  }
}

void logApInfo(const char* prefix) {
  wifi_ap_record_t apInfo;
  if (esp_wifi_sta_get_ap_info(&apInfo) == ESP_OK) {
    Serial.printf("%s BSSID %02x:%02x:%02x:%02x:%02x:%02x, CH %d, RSSI %d, AUTH %s\n",
                  prefix,
                  apInfo.bssid[0], apInfo.bssid[1], apInfo.bssid[2],
                  apInfo.bssid[3], apInfo.bssid[4], apInfo.bssid[5],
                  apInfo.primary, apInfo.rssi, wifiAuthModeName(apInfo.authmode));
  } else {
    Serial.printf("%s AP-Info nicht verfügbar\n", prefix);
  }
}

void maintainWiFi() {
  if (configPortalActive) return;
  static const unsigned long WIFI_RECONNECT_INTERVAL = 15000UL;
  static const unsigned long WIFI_MAX_OUTAGE = 300000UL; // 5 minutes
  static const unsigned long WIFI_CONNECT_GRACE = 10000UL; // 10 seconds
  unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    lastWiFiConnected = now;
    return;
  }

  if (lastWiFiConnectEvent != 0 && (now - lastWiFiConnectEvent) < WIFI_CONNECT_GRACE) {
    return;
  }

  if (!wifiReconnectIssued || (now - lastWiFiReconnectAttempt) > WIFI_RECONNECT_INTERVAL) {
    Serial.println("WiFi offline → attempting reconnect");
    Serial.printf("WiFi reconnect SSID: %s\n", WiFi.SSID().c_str());
    wifiReconnectIssued = true;
    lastWiFiReconnectAttempt = now;
    WiFi.reconnect();
  }

  if (now - lastWiFiConnected > WIFI_MAX_OUTAGE) {
    Serial.println("WiFi offline for more than 5 minutes → restarting ESP");
    ESP.restart();
  }
}

void configureWiFiManager(WiFiManager& wm) {
  wm.setSaveParamsCallback(saveParamCallback);
  setupWiFiManagerParams(wm);
  const char* menuItems[] = {"wifi","param","info","restart","exit","update"};
  wm.setMenu(menuItems, sizeof(menuItems)/sizeof(menuItems[0]));
  wm.setConfigPortalTimeout(60);
}

void setupWiFiManagerParams(WiFiManager& wm) {
  for (size_t i=0; i<sizeof(paramInfos)/sizeof(paramInfos[0]); i++) {
    new(&paramObjects[i]) WiFiManagerParameter(
      paramInfos[i].id,
      paramInfos[i].label,
      paramInfos[i].val.c_str(),
      paramInfos[i].len,
      paramInfos[i].html
    );
    wm.addParameter(&paramObjects[i]);
  }
}

bool syncConfigFromWiFiManager() {
  bool changed = false;
  for (size_t i = 0; i < sizeof(paramInfos) / sizeof(paramInfos[0]); i++) {
    const char* value = paramObjects[i].getValue();
    if (!value) continue;
    if (paramInfos[i].val != value) {
      paramInfos[i].val = value;
      changed = true;
    }
  }

  if (changed) {
    saveParamCallback();
    mqttClient.disconnect();
    mqttClient.setServer(g_mqttServer.c_str(), g_mqttPort.toInt());
    WiFi.setHostname(g_host.c_str());
    ArduinoOTA.setHostname(g_host.c_str());
    ArduinoOTA.setPassword(g_otaPass.c_str());
    Serial.println("Konfiguration aus Portal übernommen");
  }
  return changed;
}

// ---------------------------------------------------------
// setup()
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(RADAR_BOOT_PIN, INPUT_PULLUP);

  // Load preferences
  prefs.begin("myRadar", true);
  g_mqttServer = prefs.getString("mqtt_server", g_mqttServer);
  g_mqttPort   = prefs.getString("mqtt_port",   g_mqttPort);
  g_mqttTopic  = prefs.getString("mqtt_topic",  g_mqttTopic);
  g_radarRxPin = prefs.getString("radar_rx",    g_radarRxPin);
  g_radarTxPin = prefs.getString("radar_tx",    g_radarTxPin);
  g_host       = prefs.getString("host",        g_host);
  g_otaPass    = prefs.getString("otaPass",     g_otaPass);
  prefs.end();

  // WiFi setup
  WiFiManager wm;
  configureWiFiManager(wm);
  bool ok = wm.autoConnect("AutoConnectAP","12345678");
  Serial.println(ok ? "WiFi verbunden" : "WiFiManager Timeout");
  if (ok) WiFi.setTxPower(WIFI_POWER_19_5dBm);

  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  esp_wifi_set_ps(WIFI_PS_NONE);
  lastWiFiConnected = millis();
  wifiReconnectIssued = false;
  syncConfigFromWiFiManager();

  // SICHERHEIT: Einfacher Event-Handler ohne WiFi-API Aufrufe
  // WiFi.setAutoReconnect(true) macht den Reconnect automatisch
  WiFi.onEvent(
    [](arduino_event_id_t evt, WiFiEventInfo_t info){
      switch (evt) {
        case ARDUINO_EVENT_WIFI_STA_START:
          Serial.println("WiFi event: STA_START");
          break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
          Serial.printf("WiFi event: CONNECTED to %s\n",
                        reinterpret_cast<const char*>(info.wifi_sta_connected.ssid));
          logApInfo("WiFi AP");
          lastWiFiConnectEvent = millis();
          wifiReconnectIssued = false;
          lastWiFiReconnectAttempt = 0;
          break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          Serial.printf("WiFi event: GOT_IP %s\n", WiFi.localIP().toString().c_str());
          Serial.printf("WiFi IP-Info: GW %s, MASK %s, DNS1 %s, DNS2 %s\n",
                        WiFi.gatewayIP().toString().c_str(),
                        WiFi.subnetMask().toString().c_str(),
                        WiFi.dnsIP(0).toString().c_str(),
                        WiFi.dnsIP(1).toString().c_str());
          logApInfo("WiFi AP");
          lastWiFiConnected = millis();
          lastWiFiConnectEvent = 0;
          wifiReconnectIssued = false;
          lastWiFiReconnectAttempt = 0;
          break;
        case ARDUINO_EVENT_WIFI_STA_LOST_IP:
          Serial.println("WiFi event: LOST_IP");
          break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          Serial.printf("WiFi event: DISCONNECTED (reason=%d %s)\n",
                        info.wifi_sta_disconnected.reason,
                        wifiDisconnectReason(info.wifi_sta_disconnected.reason));
          Serial.printf("WiFi SSID: %s\n", WiFi.SSID().c_str());
          Serial.printf("WiFi status: %d, seit letzter Verbindung %lu ms, letzter Reconnect %lu ms\n",
                        WiFi.status(),
                        millis() - lastWiFiConnected,
                        millis() - lastWiFiReconnectAttempt);
          logApInfo("WiFi last AP");
          wifiReconnectCount++;
          wifiReconnectIssued = true;
          lastWiFiReconnectAttempt = millis();
          break;
        default:
          Serial.printf("WiFi event: %d\n", evt);
          break;
      }
    }
  );

  otaSetup();
  WiFi.setHostname(g_host.c_str());
  Serial.println("OTA und Hostname konfiguriert");

  // Radar serial setup
  Serial.print("Starte Radar auf RX=");
  Serial.print(g_radarRxPin);
  Serial.print(" TX=");
  Serial.println(g_radarTxPin);
  Serial1.begin(256000, SERIAL_8N1,
                g_radarRxPin.toInt(),
                g_radarTxPin.toInt());
  delay(100);

  Serial.println("Setze Radar-Parameter...");
  setMaxRadarRange(g_maxRangeMeters);
  enableMultiTargetMode();
  Serial.println("Radar konfiguriert");

  // MQTT setup
  Serial.print("MQTT Server: ");
  Serial.print(g_mqttServer);
  Serial.print(":");
  Serial.println(g_mqttPort);
  mqttClient.setServer(g_mqttServer.c_str(), g_mqttPort.toInt());
  mqttClient.setCallback(mqttCallback);

  // SICHERHEIT: Größere Buffer und längere Keep-Alive
  mqttClient.setBufferSize(512);  // Größerer Buffer für JSON
  mqttClient.setKeepAlive(60);    // 60 Sekunden Keep-Alive (statt 15)
  mqttClient.setSocketTimeout(15); // 15 Sekunden Socket-Timeout

  mqttReconnect();

  // WebServer setup
  setupWebServer();
  Serial.print("WebServer gestartet: http://");
  Serial.println(WiFi.localIP());

  lastRadarDataTime = millis();
  Serial.println("Setup abgeschlossen - starte Loop");
}

// ---------------------------------------------------------
// loop()
// ---------------------------------------------------------
void loop() {
  // Check BOOT button for config portal (hold for 3 seconds)
  static unsigned long bootPressStart = 0;
  static bool bootPressed = false;

  if (digitalRead(RADAR_BOOT_PIN) == LOW) {
    if (!bootPressed) {
      bootPressed = true;
      bootPressStart = millis();
    } else if (millis() - bootPressStart > 3000) {
      Serial.println("BOOT button pressed - starting config portal");
      startConfigPortal = true;
      bootPressed = false;
    }
  } else {
    bootPressed = false;
  }

  // WiFi connection monitoring (Auto-reconnect ist aktiv via setAutoReconnect)
  maintainWiFi();
  if (WiFi.status() != WL_CONNECTED &&
      millis() - lastWiFiCheck > 10000UL) {
    Serial.print("WiFi not connected, status: ");
    Serial.println(WiFi.status());
    lastWiFiCheck = millis();
  }

  // OTA handling
  ArduinoOTA.handle();
  if (otaInProgress) return;

  // WebServer
  handleWebServer();

  // MQTT
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiConnected) {
    if (!mqttClient.connected()) mqttReconnect();
    mqttClient.loop();
  }

  // Radar data
  readRadarData();

  // Publishing
  unsigned long now = millis();
  if (wifiConnected) {
    if (now - lastRadarPub >= RADAR_INTERVAL_MS) {
      lastRadarPub = now;
      publishRadarJson();
    }
    if (now - lastStatusPub >= STATUS_INTERVAL) {
      lastStatusPub = now;
      publishStatus();
    }
  }

  // Command handling
  if (wifiConnected) {
    handleMqttCommands();
  }
  checkRadarConnection();

  delay(1);
}

// ---------------------------------------------------------
// handleMqttCommands()
// ---------------------------------------------------------
void handleMqttCommands() {
  if (startConfigPortal) {
    startConfigPortal = false;
    Serial.println("MQTT 'config' → öffne Config‑Portal");
    configPortalActive = true;

    stopWebServer();
    mqttClient.disconnect();

    WiFiManager wm2;
    configureWiFiManager(wm2);
    bool portalOk = wm2.startConfigPortal("OnDemandAP","12345678");
    configPortalActive = false;

    bool updated = syncConfigFromWiFiManager();
    setupWebServer();
    Serial.println("Portal beendet → reconnect WIFI");
    if (!portalOk) {
      WiFi.reconnect();
    }
    wifiReconnectIssued = false;
    lastWiFiReconnectAttempt = 0;
    if (updated || portalOk) {
      mqttReconnect();
    }
  }
  if (rebootRequested) {
    rebootRequested = false;
    ESP.restart();
  }
}
