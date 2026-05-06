#pragma once

#include <Arduino.h>

#define SP_START 0x07e
#define SP_ESCAPE 0x07d
#define VALID_SENSOR_PACKAGE 0x10
#define kSmartPortBaud 57600
#define VALID_DATA_INTERVAL_MS 1
#define SIZE_RX_BUFFER 9
#define SIZE_TX_PACKET 8
#define degree_in_minutes_div10k 600000

class SmartPortSlave {
 public:
  SmartPortSlave();
  ~SmartPortSlave();

  // rxPin/txPin/invertEsp32 are used on ESP32. On other platforms they are ignored.
  void begin(
      HardwareSerial* smartPortSerial,
      uint8_t mySP_ID = 0x6a,
      Print* debugSerial = nullptr,
      int8_t rxPin = -1,
      int8_t txPin = -1,
      bool invertEsp32 = true);

  void setMySP_ID(uint8_t id);
  void update(void);
  bool sniffedDataAvailable();
  void getSniffedData(char* rx_buffer);
  bool txBufferEmpty();
  void sendSensorWord(uint16_t sensorID, uint32_t sensorValue);
  void sendV_A3(int32_t v_a3_value, int16_t sensorID);
  void sendLON(int32_t lon_min_X10k, int16_t sensorID);
  void sendLAT(int32_t lat_min_X10k, int16_t sensorID);
  void sendDate(uint8_t year, uint8_t month, uint8_t day, int16_t sensorID);
  void sendDate(char* date_dmy, int16_t sensorID);
  void sendTime(uint8_t hour, uint8_t minute, uint8_t second, int16_t sensorID);
  void sendTime(char* time_hms, int16_t sensorID);
  void sendCellVoltage(uint32_t voltage_mV, uint8_t cellNr, int16_t sensorID);
  void sendAltitude(int32_t altitude_m, int16_t sensorID);
  void sendHeading(uint32_t heading_centiDeg, int16_t sensorID);
  void sendGnssAltitude(int32_t altitude_cm, int16_t sensorID);
  void sendGnssSpeed(int32_t speed_mm_s, int16_t sensorID);

 private:
  bool m_sensorTxPacketReady;
  void clearReadBuffer();
  void outputSmartPortChar(uint8_t character);
  char getInputSmartPortChar();
  void setCRC(void);
  void receiveSmartPortChar(void);
  void answer_SmartPort(void);
  void sendMyNextSmartPortData(bool validData);
  void setSensorValue(uint8_t start, uint8_t length, uint32_t sensorValue);
  void setSensorID(uint16_t sensorID);
  void emptyRXBuffer(HardwareSerial* serial = nullptr);

  Print* m_debugSerial;
  HardwareSerial* m_smartPortSerial;
  uint32_t m_LastTimeNow;
  uint32_t m_sensorPacketNr;
  char* m_rxBuffer;
  char* m_rxBuf2;
  char* m_txPacket;
  char* m_emptyTxPacket;
  char* m_UartTxBuffer;
  uint8_t m_mySP_ID;
  uint8_t m_txPacketIndex;
  uint8_t m_readBufIndex;
  uint32_t guard;
  uint8_t m_TxMaxAvail;
  uint8_t m_availableTX;
  bool m_TxMode;
  bool m_EscapeNext;
  bool m_sniffedDataReady;
  char iFormatedString[100];
};
