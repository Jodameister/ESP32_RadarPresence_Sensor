# ESP32 Radar Presence Sensor

A robust ESP32-based presence detection system using the RD-03D radar sensor with MQTT integration and OTA update support.

## Features

- **Multi-Target Detection**: Track up to 3 targets simultaneously with position, speed, and distance
- **MQTT Integration**: Real-time data publishing and remote command control
- **WiFi Manager**: Easy configuration through web portal
- **OTA Updates**: Wireless firmware updates
- **Exponential Smoothing**: Signal filtering for stable readings
- **Dynamic Hold Interval**: Configurable presence hold time
- **Configurable Detection Range**: Adjustable via MQTT commands
- **Auto-Reconnect**: Robust WiFi and MQTT connection management
- **Status Monitoring**: Comprehensive system health reporting
- **Live Web Dashboard**: Browser UI streams radar & ESP telemetry via SSE

## Hardware Requirements

- ESP32 development board
- RD-03D Radar Sensor (256000 baud UART)
- MQTT Broker (e.g., Mosquitto)

## Default Pin Configuration

- **Radar RX**: GPIO 16
- **Radar TX**: GPIO 17
- **BOOT Button**: GPIO 0 (config portal trigger)

## Installation

### 1. Prerequisites

Install the following libraries via Arduino Library Manager:

- WiFiManager by tzapu
- PubSubClient by Nick O'Leary
- ArduinoJson by Benoit Blanchon

### 2. Configuration

First boot will create a WiFi Access Point:
- **SSID**: `AutoConnectAP`
- **Password**: `12345678`

Connect and configure:
- WiFi credentials
- MQTT server IP and port
- MQTT topic prefix
- Radar RX/TX pins
- Device hostname
- OTA password

### 3. Upload

1. Open `RadarPresence.ino` in Arduino IDE
2. Select ESP32 board
3. Upload sketch

## MQTT Topics

### Published Topics

#### `<topic>` - Radar Data
Published every 100ms when targets detected, max 1/second when no targets:

```json
{
  "targetCount": 1,
  "target1": {
    "presence": true,
    "x": 120,
    "y": -45,
    "speed": 5,
    "distRaw": 130,
    "distance": 128,
    "angleDeg": -20.5
  },
  "target2": {
    "presence": false
  },
  "target3": {
    "presence": false
  }
}
```

#### `<topic>/status` - System Status
Published every 10 seconds:

```json
{
  "fwVersion": "v1.7",
  "uptime_min": 42,
  "resetReason": "software",
  "rssi": -67,
  "channel": 6,
  "heap_free": 234567,
  "temp_c": 45.5,
  "mqttState": 0,
  "wifiReconnects": 0,
  "radarTimeouts": 0,
  "radarSerialRestarts": 1,
  "lastRadarDelta": 23,
  "holdMs": 500,
  "range_m": 2.1
}
```

### Subscribe Topics

#### `<topic>/cmd` - Commands

Available commands:

| Command | Description | Example |
|---------|-------------|---------|
| `config` | Open WiFi config portal | `config` |
| `reboot` | Restart ESP32 | `reboot` |
| `resetRadar` | Restart radar serial connection | `resetRadar` |
| `setRange:<meters>` | Set detection range (0.7-15m) | `setRange:4` |
| `setHold:<ms>` | Set hold interval (0-10000ms) | `setHold:1000` |
| `getStatus` | Request immediate status update | `getStatus` |

#### `<topic>/ack` - Command Acknowledgments

Receives confirmation for executed commands.

## Configuration Portal

### Via BOOT Button
Hold BOOT button for 3 seconds to open on-demand config portal:
- **SSID**: `OnDemandAP`
- **Password**: `12345678`

### Via MQTT
Send `config` command to `<topic>/cmd`

## Architecture

The codebase is modular for maintainability:

```
RadarPresence/
├── RadarPresence.ino    # Main entry point, WiFi & loop
├── Config.h/cpp         # Global configuration & variables
├── RadarHandler.h/cpp   # Radar communication & data processing
├── MQTTHandler.h/cpp    # MQTT client & command handling
├── OTAHandler.h/cpp     # OTA update management
└── WebServerHandler.h/cpp # Web dashboard, API, and SSE streaming
```

## Web Dashboard

- Aufruf über `http://<hostname-oder-ip>/` (Hostname wird im WiFiManager gesetzt)
- Frontend streamt Live-Daten via Server-Sent Events (`/events`) und fällt bei Bedarf auf 1 s HTTP-Polling (`/api/radar`) zurück
- Buttons erlauben Neustart von ESP, Radar sowie das Öffnen des WiFiManager-Portals
- Warnungen markieren schwaches WLAN, wenig Heap oder ausstehende Radarframes

## Safety Features

### Buffer Overflow Protection
- Bounds checking on all array accesses
- Pre-write buffer overflow detection
- Frame size validation before processing

### MQTT Stability
- Non-blocking reconnection (5s interval)
- Extended keep-alive (60s vs 15s default)
- Larger message buffer (512 bytes)
- Last Will Testament for disconnect detection
- Publish error handling

### Radar Robustness
- Automatic serial reset on data timeout (3s)
- ESP32 restart on failed recovery (30s)
- Frame deduplication to reduce processing
- Configurable read limits to prevent blocking

## Troubleshooting

### No Radar Data
Check serial monitor for:
- `"Keine Radar-Daten empfangen"` - Check wiring
- `"WARN: radarBuf overflow reset"` - Excessive data rate
- `"WARN: Invalid frame size"` - Communication error

Send MQTT command: `resetRadar`

### MQTT Connection Lost
Monitor shows:
- `"MQTT reconnect... FAILED, rc=X"` - Check broker availability
- Error codes: -4 = timeout, -2 = connection refused

Configuration:
- Keep-alive: 60 seconds
- Reconnect interval: 5 seconds
- Buffer size: 512 bytes

### WiFi Disconnections
- Auto-reconnect enabled
- Check `wifiReconnects` in status
- Power supply stability important

## Performance Characteristics

- **Radar Update Rate**: 100ms
- **Status Update Rate**: 10s
- **Zero Target Rate Limit**: 1/second
- **MQTT Keep-Alive**: 60s
- **Radar Timeout**: 3s
- **Serial Reset Timeout**: 30s

## Radar Data Processing

### Smoothing
Exponential moving average with α = 0.4:
- Reduces noise while maintaining responsiveness
- Applied to X, Y, and speed values
- Raw distance always current

### Hold Interval
Configurable persistence after target disappears:
- Default: 500ms
- Range: 0-10000ms
- Prevents flickering on brief signal loss

### Detection Range
Configurable via MQTT:
- Default: 2.1m
- Range: 0.7-10.5m (1-15 gates × 0.7m)
- Applied immediately to radar sensor

## Version History

### v1.5.d (Current)
- Modular code architecture
- Critical safety improvements
- Enhanced MQTT stability
- Non-blocking operations
- Comprehensive error handling

### v1.3e
- Original monolithic implementation
- Basic MQTT and radar functionality

## License

This project is provided as-is for educational and personal use.

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Test thoroughly on hardware
4. Submit pull request with detailed description

For detailed coding, build, and commit conventions see `AGENTS.md`.

## Author

Developed with assistance from Claude Code.
