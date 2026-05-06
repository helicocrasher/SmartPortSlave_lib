
# SmartPortSlave

FrSky SmartPort half-duplex slave helper for Arduino-compatible targets using PlatformIO or the Arduino build flow.

## Features

- Sends SmartPort telemetry fields from a slave device.
- Supports ESP32 and STM32 Arduino targets with target-specific serial setup in `begin(...)`.
- Keeps the public header in `inc/` and implementation in `src/` for PlatformIO packaging.

## Installation

### PlatformIO Registry

After publication, add this to your `lib_deps`:

```ini
lib_deps =
  helicocrasher/SmartPortSlave
```

### Git Dependency

Until a registry release is published, use the Git repository directly:

```ini
lib_deps =
  https://github.com/helicocrasher/SmartPortSlave_lib.git
```

## Basic Usage

```cpp
#include <Arduino.h>
#include <SmartPortSlave.h>

SmartPortSlave smartPort;

constexpr uint8_t kSensorId = 0x6A;
constexpr uint16_t kFieldId = 0x0901;

#if defined(ARDUINO_ARCH_ESP32)
constexpr int8_t kSmartPortRxPin = 5;
constexpr int8_t kSmartPortTxPin = 3;
HardwareSerial& spSerial = Serial0;
#else
HardwareSerial& spSerial = Serial1;
#endif

void setup() {
#if defined(ARDUINO_ARCH_ESP32)
  smartPort.begin(&spSerial, kSensorId, nullptr, kSmartPortRxPin, kSmartPortTxPin, true);
#else
  smartPort.begin(&spSerial, kSensorId, nullptr);
#endif
}

void loop() {
  smartPort.update();

  static uint32_t lastMs = 0;
  static uint32_t counter = 0;

  if (millis() - lastMs >= 200) {
    smartPort.sendSensorWord(kFieldId, counter++);
    lastMs = millis();
  }
}
```

## Examples

- `examples/tiny_cross_arch`: minimal cross-architecture example with debug serial output.
- `examples/tiny_minimal_no_debug`: minimal example without debug serial output.

## Target Notes

- ESP32 uses the extended `begin(...)` overload with explicit RX/TX pin selection and optional signal inversion.
- STM32 uses Arduino `HardwareSerial` half-duplex support where available.
- Other Arduino targets fall back to standard `HardwareSerial::begin(57600)`.

## Publishing Notes

This repository includes a PlatformIO `library.json` manifest for publishing through `pio pkg publish --type library`.
