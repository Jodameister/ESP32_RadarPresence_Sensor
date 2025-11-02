//************************************************************
// ESP32 RadarPresence – Main sketch
//************************************************************

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>

#include "Config.h"
#include "RadarHandler.h"
#include "MQTTHandler.h"
#include "OTAHandler.h"

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

  // SICHERHEIT: Einfacher Event-Handler ohne WiFi-API Aufrufe
  // WiFi.setAutoReconnect(true) macht den Reconnect automatisch
  WiFi.onEvent(
    [](arduino_event_id_t evt, WiFiEventInfo_t info){
      if (evt == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
        Serial.println("WiFi disconnected");
        wifiReconnectCount++;
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
  if (WiFi.status() != WL_CONNECTED &&
      millis() - lastWiFiCheck > 10000UL) {
    Serial.print("WiFi not connected, status: ");
    Serial.println(WiFi.status());
    lastWiFiCheck = millis();
  }

  // OTA handling
  ArduinoOTA.handle();
  if (otaInProgress) return;

  // MQTT
  if (!mqttClient.connected()) mqttReconnect();
  mqttClient.loop();

  // Radar data
  readRadarData();

  // Publishing
  unsigned long now = millis();
  if (now - lastRadarPub >= RADAR_INTERVAL_MS) {
    lastRadarPub = now;
    publishRadarJson();
  }
  if (now - lastStatusPub >= STATUS_INTERVAL) {
    lastStatusPub = now;
    publishStatus();
  }

  // Command handling
  handleMqttCommands();
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
    WiFiManager wm2;
    configureWiFiManager(wm2);
    wm2.startConfigPortal("OnDemandAP","12345678");
    Serial.println("Portal beendet → reconnect WIFI");
    WiFi.reconnect();
  }
  if (rebootRequested) {
    rebootRequested = false;
    ESP.restart();
  }
}
