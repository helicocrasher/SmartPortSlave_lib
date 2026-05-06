SmartPortSlave tiny minimal no-debug example

File:
- tiny_minimal_no_debug.ino

Purpose:
- Smallest practical SmartPortSlave usage.
- No Serial.begin(), no debug stream output.

Architecture handling:
- STM32: begin(serial, id, nullptr)
- ESP32: begin(serial, id, nullptr, rxPin, txPin, invert)

Notes:
- For ESP32, update kSmartPortRxPin and kSmartPortTxPin to match your board/wiring.
- Uses Serial0 on ESP32 and Serial1 on STM32 by default.
