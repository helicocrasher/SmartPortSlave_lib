#include <Arduino.h>
#include <SmartPortSlave.h>

namespace {

constexpr uint8_t kSensorId = 0x6A;
constexpr uint16_t kFieldId = 0x0901;

SmartPortSlave smartPort;

#if defined(ARDUINO_ARCH_ESP32)
// Adjust these pins in your project if your wiring differs.
constexpr int8_t kSmartPortRxPin = 5;
constexpr int8_t kSmartPortTxPin = 3;
HardwareSerial& spSerial = Serial0;
#else
// STM32 Arduino core usually provides Serial1 for external UART usage.
HardwareSerial& spSerial = Serial1;
#endif

}  // namespace

void setup() {
  Serial.begin(115200);

#if defined(ARDUINO_ARCH_ESP32)
  smartPort.begin(&spSerial, kSensorId, &Serial, kSmartPortRxPin, kSmartPortTxPin, true);
#else
  smartPort.begin(&spSerial, kSensorId, &Serial);
#endif

  Serial.println("SmartPort tiny cross-arch example started");
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
