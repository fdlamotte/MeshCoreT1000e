#include <BaseCompanionRadioMesh.h>
#include "gps.h"

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

#ifdef ESP32
  #ifdef BLE_PIN_CODE
    #include <helpers/esp32/SerialBLEInterface.h>
    SerialBLEInterface serial_interface;
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#elif defined(NRF52_PLATFORM)
  #ifdef BLE_PIN_CODE
    #include <helpers/nrf52/SerialBLEInterface.h>
    SerialBLEInterface serial_interface;
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#else
  #error "need to define a serial interface"
#endif

#if defined(NRF52_PLATFORM)
RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);
#elif defined(P_LORA_SCLK)
SPIClass spi;
RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
#else
RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY);
#endif
StdRNG fast_rng;
SimpleMeshTables tables;

class T1000eMesh : public BaseCompanionRadioMesh {
  bool gps_time_sync_needed = true;
  MicroNMEA* _nmea;

public:
  T1000eMesh(RADIO_CLASS& phy, RadioLibWrapper& rw, mesh::RNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables, MicroNMEA& nmea)
     : BaseCompanionRadioMesh(phy, rw, rng, rtc, tables), _nmea(&nmea) {
  
     }

  void loop() {
    BaseCompanionRadioMesh::loop();
    static long next_gps_update = 0;
    if (millisHasNowPassed(next_gps_update)) {
      if (_nmea->isValid()) {
        _prefs.node_lat = ((double)_nmea->getLatitude())/1000000.;
        _prefs.node_lon = ((double)_nmea->getLongitude())/1000000.;
        if (gps_time_sync_needed) {
          DateTime dt(_nmea->getYear(), _nmea->getMonth(),_nmea->getDay(),_nmea->getHour(),_nmea->getMinute(),_nmea->getSecond());
          getRTCClock()->setCurrentTime(dt.unixtime());
          gps_time_sync_needed = false;
        }
      }
      next_gps_update = futureMillis(5000);
    } 
  }
};

T1000eMesh the_mesh(radio, *new WRAPPER_CLASS(radio, board), fast_rng, *new VolatileRTCClock(), tables, nmea);


void halt() {
  while (1) ;
}

void setup() {

  Serial.begin(115200);
  board.begin();

#ifdef SX126X_DIO3_TCXO_VOLTAGE
  float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 1.6f;
#endif

  SPI.setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
  SPI.begin();

  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_TX_POWER, 8, tcxo);
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    halt();
  }

  radio.setCRC(0);
  fast_rng.begin(radio.random(0x7FFFFFFF));
  RadioNoiseListener trng(radio);

  InternalFS.begin();
  the_mesh.begin(InternalFS, trng);

  char dev_name[32+10];
  sprintf(dev_name, "MeshCore-%s", the_mesh.getNodeName());
  serial_interface.begin(dev_name, BLE_PIN_CODE);

  the_mesh.startInterface(serial_interface);

  // GPS Setup
  digitalWrite(PIN_GPS_EN, GPS_EN_ACTIVE);
  gps_setup();
}

void loop() {
  gps_feed_nmea();

  static int nextCheck = 0;
  if (the_mesh.millisHasNowPassed(nextCheck)) {
    gps_loop();
    nextCheck = the_mesh.futureMillis(5000);
  }

  the_mesh.loop();
}
