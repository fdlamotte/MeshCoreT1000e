#pragma once

#include <helpers/BaseSerialInterface.h>

#include <bluefruit.h>


class SerialBLEInterface : public BaseSerialInterface {
  uint32_t _pin_code;
  bool _isEnabled;
  bool deviceConnected;
  unsigned long _last_write;

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  #define FRAME_QUEUE_SIZE  4
  int recv_queue_len;
  Frame recv_queue[FRAME_QUEUE_SIZE];
  int send_queue_len;
  Frame send_queue[FRAME_QUEUE_SIZE];

  void clearBuffers() { recv_queue_len = 0; send_queue_len = 0; }

  BLEDfu bledfu;
  BLEUart bleuart;

public:
  SerialBLEInterface() {
    deviceConnected = false;
    _isEnabled = false;
    _last_write = 0;
    clearBuffers();
  }

  void begin(const char* device_name, uint32_t pin_code);
  void startAdv(void);

  // BaseSerialInterface methods
  void enable() override;
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }

  bool isConnected() const override;

  bool isWriteBusy() const override;
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[]) override;

  virtual void onWrite(uint16_t conn_hdl);

  static void rx_callback(uint16_t conn_hdl);
  static void connect_callback(uint16_t conn_hdl);
  static void disconnect_callback(uint16_t conn_hdl, uint8_t reason);
};


#if BLE_DEBUG_LOGGING && ARDUINO
  #include <Arduino.h>
  #define BLE_DEBUG_PRINT(F, ...) Serial.printf("BLE: " F, ##__VA_ARGS__)
  #define BLE_DEBUG_PRINTLN(F, ...) Serial.printf("BLE: " F "\n", ##__VA_ARGS__)
#else
  #define BLE_DEBUG_PRINT(...) {}
  #define BLE_DEBUG_PRINTLN(...) {}
#endif
