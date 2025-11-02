# Repository Guidelines

## Project Structure & Module Organization
- `RadarPresence.ino` orchestrates Wi-Fi setup, OTA bootstrapping, radar serial init, MQTT wiring, and the main loop.
- `Config.{h,cpp}` stores global configuration, persisted preferences, and runtime flags such as `otaInProgress`.
- `RadarHandler.{h,cpp}`, `MQTTHandler.{h,cpp}`, `OTAHandler.{h,cpp}`, and `WebServerHandler.{h,cpp}` isolate sensor IO, broker interactions, OTA callbacks, and the HTTP status interface.
- Keep new modules beside their headers (`FooHandler.cpp/.h`) to preserve the flat structure. Place shared constants or types in `Config.h` unless they are handler-specific.

## Build, Test, and Development Commands
- `arduino-cli compile -b esp32:esp32:esp32 RadarPresence.ino` builds the sketch; install `arduino-cli` and select the `esp32` core 3.x toolchain first.
- `arduino-cli upload -p /dev/ttyUSB0 -b esp32:esp32:esp32 RadarPresence.ino` flashes via serial; adjust the port to match your board.
- `arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200` opens the serial monitor for runtime diagnostics.
- OTA updates: run `ArduinoOTA.begin()` already wired in `otaSetup()`. Trigger uploads from the Arduino IDE by selecting “Network Ports” after the device advertises via mDNS.

## Coding Style & Naming Conventions
- Use 2-space indentation, no tabs. Keep braces on the same line as control statements (`if (...) {`).
- Follow the existing snake_case for global `String` holders and camelCase for functions/locals.
- Include the specific ESP32 core headers that a file depends on (e.g., `#include <SHA2Builder.h>` when relying on OTA password hashing).
- Favor concise lambdas for event handlers and keep logging human-readable (`Serial.println` in German is already common).

## Testing Guidelines
- No automated test harness exists; rely on bench testing with the RD-03D radar. Validate MQTT flows against a broker (e.g., Mosquitto) and verify OTA by performing at least one password-protected upload.
- Serial monitor output should show “Setup abgeschlossen - starte Loop” and MQTT status publishes every 10 s. Investigate if `wifiReconnectCount` or `radarTimeouts` increment unexpectedly.

## Commit & Pull Request Guidelines
- Recent history mixes Conventional Commit prefixes (`feat:`, `fix:`) with sentence-style messages. Prefer Conventional Commits for clarity (`feat: adjust radar debounce`).
- Scope commits narrowly (one handler or feature at a time) and reference the modified files in the body.
- Pull requests should include: purpose summary, manual test notes (Wi-Fi, MQTT, OTA), screenshots/log excerpts where relevant, and links to related issues or forum threads.
