SmartPortSlave tiny cross-arch example

File:
- tiny_cross_arch.ino

What it shows:
- Basic SmartPortSlave initialization and periodic telemetry send.
- Architecture-dependent begin() usage:
  - STM32: begin(serial, id, debug)
  - ESP32: begin(serial, id, debug, rxPin, txPin, invert)

Notes:
- For ESP32, adjust kSmartPortRxPin and kSmartPortTxPin to your board wiring.
- This example uses Serial0 on ESP32 and Serial1 on STM32 by default.
