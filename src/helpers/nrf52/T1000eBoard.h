#pragma once

#include <MeshCore.h>
#include <Arduino.h>

// LoRa pins from variant.h
#define  P_LORA_DIO_1   LORA_DIO1
#define  P_LORA_NSS     LORA_CS
#define  P_LORA_RESET  LORA_RESET
#define  P_LORA_BUSY    LORA_DIO2
#define  P_LORA_SCLK    PIN_SPI_SCK
#define  P_LORA_MISO    PIN_SPI_MISO
#define  P_LORA_MOSI    PIN_SPI_MOSI
#define  LR1110_POWER_EN  37
 
// already defined in variant.h
//#define LR11X0_DIO_AS_RF_SWITCH  true
//#define LR11X0_DIO3_TCXO_VOLTAGE   1.6

// built-ins
//#define  PIN_VBAT_READ    5
//#define  ADC_MULTIPLIER   (3 * 1.73 * 1000)

class T1000eBoard : public mesh::MainBoard {
protected:
  uint8_t startup_reason;

public:
  void begin() {
    // for future use, sub-classes SHOULD call this from their begin()
    startup_reason = BD_STARTUP_NORMAL;

//    pinMode(PIN_VBAT_READ, INPUT);

// Doesn't seem to be a pwr en pin ...
//    pinMode(LR1110_POWER_EN, OUTPUT);
//    digitalWrite(LR1110_POWER_EN, HIGH);
    delay(10);   // give sx1262 some time to power up
  }

  uint16_t getBattMilliVolts() override {
    return 0;
  }

  uint8_t getStartupReason() const override { return startup_reason; }

  const char* getManufacturerName() const override {
    return "Seeed Tracker T1000-e";
  }

  void reboot() override {
    NVIC_SystemReset();
  }

//  bool startOTAUpdate() override;
};
