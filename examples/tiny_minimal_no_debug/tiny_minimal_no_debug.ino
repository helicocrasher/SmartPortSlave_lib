#include <Arduino.h>
#include <SmartPortSlave.h>

namespace {

constexpr uint8_t kSensorId = 0x6A;
constexpr uint16_t kFieldId = 0x0901;

SmartPortSlave smartPort;

#if defined(ARDUINO_ARCH_ESP32)
// Set these for your board wiring.
constexpr int8_t kSmartPortRxPin = 5;
constexpr int8_t kSmartPortTxPin = 3;
HardwareSerial& spSerial = Serial0;
#else
HardwareSerial& spSerial = Serial1;
#endif

}  // namespace

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
