#include <Arduino.h>
#include <bluefruit.h>
#include "gps.h"

#define _PWM_LOGLEVEL_       4

#define USING_TIMER false
#include "nRF52_PWM.h"

#include <helpers/nrf52/T1000eBoard.h>
#include <helpers/CustomLR1110Wrapper.h>

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     250
#endif
#ifndef LORA_SF
  #define LORA_SF     10
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif
#ifndef MAX_LORA_TX_POWER
  #define MAX_LORA_TX_POWER  30
#endif

#define  PUBLIC_GROUP_PSK  "izOH6cXN6mrJ5e26oRXNcg=="

#include <helpers/BaseCompanionRadioMesh.h>

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

#ifdef BLE_PIN_CODE
  #include <helpers/nrf52/SerialBLEInterface.h>
  SerialBLEInterface serial_interface;
#else
  #include <helpers/ArduinoSerialInterface.h>
  ArduinoSerialInterface serial_interface;
#endif

static T1000eBoard board;

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);
StdRNG fast_rng;
SimpleMeshTables tables;

class T1000eMesh : public BaseCompanionRadioMesh {
  bool gps_time_sync_needed = true;
  MicroNMEA* _nmea;
  nRF52_PWM _pwm;

public:
  T1000eMesh(RADIO_CLASS& phy, RadioLibWrapper& rw, mesh::RNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables, MicroNMEA& nmea)
     : BaseCompanionRadioMesh(phy, rw, rng, rtc, tables, board, PUBLIC_GROUP_PSK, LORA_FREQ, LORA_SF, LORA_BW, LORA_CR, LORA_TX_POWER), 
     _nmea(&nmea), _pwm(nRF52_PWM(LED_PIN, 1000.0f, 100.0f)) {

     }

  void ledHandler() {
    static bool up;
    static int cycles;
    bool gps_fix = _nmea->isValid();
    bool has_msg = getUnreadMsgNb() > 0;

    static int val;
    val = (val + 5)%100;

    _pwm.setPWM(LED_PIN, 1000, val);

  }

  void buttonHandler() {
    static int lastBtnState = 0;
    static int btnPressNumber = 0;
    static int cyclesSinceBtnChange = 0;

    int btnState = digitalRead(BUTTON_PIN);
    bool btnChanged = (btnState != lastBtnState);
    if (btnChanged && (btnState == LOW)) {
      if (cyclesSinceBtnChange > 8) { // 4 sec
        _pwm.setPWM(LED_PIN, 0, 0);
        delay(10);
        board.powerOff();
      }
    }

    if (btnChanged) 
      cyclesSinceBtnChange = 0;
    else
      cyclesSinceBtnChange++;

    lastBtnState = btnState;    
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

    static int nextBtnCheck = 0;
    if (millisHasNowPassed(nextBtnCheck)) {
      buttonHandler();
      nextBtnCheck = futureMillis(500);  
    }

    static long next_led_update = 0;
    if (millisHasNowPassed(next_led_update)) {
      ledHandler();
      next_led_update = futureMillis(500);
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

  float tcxo = 1.6f;

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

  // pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, HIGH);

  // GPS Setup
  digitalWrite(GPS_EN, HIGH);
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