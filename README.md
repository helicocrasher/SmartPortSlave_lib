
# SmartPortSlave

FrSky SmartPort half-duplex slave helper for Arduino-compatible targets using PlatformIO or the Arduino build flow.
This library is well suited to create SmartPort telemetry sensors or SmartPort bus sniffers. To work it expects a SmartPort master on the bus, usually a FrSky RC receiver.  
Note: The FrSky Smartport protocol is an inverted half-duplex protocol. To work with a HW UART the used microController has to provide support for that or additional external circuitry will be required. Check target notes.

## Features

- The SmartportSlave class is designed to operate non blocking. All methods return very fast. No delay(), no flush() on non empty TX buffers used
- Listens to traffic on SmartPort bus and buffers packets received.
- Sends SmartPort telemetry fields when adressed.
- one generic 32 bit sensor transmit format is supported: sendSensorWord() -> check SmartPortSlave.h
- 10 specific sensor transmit formats are supported: [sendV_A3(), sendLON(), SendLAT(), sendDate(), SendTime(),...] -> check SmartPortSlave.h
- TX CRC and byte escaping is handled automatically
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
- To work without external circuitry newer STM32 families have to be used. STM32G series, STM32F3, STM32F7 work (STM32F1 or STM32F4 do not provide RX/TX inversion)
- ESP32 do not directly support half-duplex inverted operation. An external diode is required, anode connected to TX pin, kathode connectd to the SmartPort bus    

## SmartPort References

- https://github.com/EdgeTX/edgetx/blob/main/radio/src/telemetry/frsky_defs.h
- https://github.com/yaapu/FrskyTelemetryScript/wiki/FrSky-SPort-protocol-specs
- https://github.com/iNavFlight/inav/blob/master/docs/Telemetry.md#available-smartport-sport-sensors

## Publishing Notes

This repository includes a PlatformIO `library.json` manifest for publishing through `pio pkg publish --type library`.
