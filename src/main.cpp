#include <Arduino.h>
#include <bluefruit.h>

#ifndef GPS_RESET_FORCE
#define GPS_RESET_FORCE HIGH
#endif

#include "helpers/MicroNMEALocationProvider.h"

//#define _PWM_LOGLEVEL_       4

#define USING_TIMER false
#include "nRF52_PWM.h"

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


#define CMD_CLI_COMMAND         50
#define RESP_CODE_CLI_RESPONSE  50

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

#ifndef BLE_ACTIVE_STATE_DURATION
  #define BLE_ACTIVE_STATE_DURATION (0)
#endif

StdRNG fast_rng;
SimpleMeshTables tables;

class T1000eMesh : public BaseCompanionRadioMesh {
  bool gps_time_sync_needed = true;
  LocationProvider* _nmea;
  nRF52_PWM _pwm;

  enum {SLEEP=0, ACTIVE=1} state;
  uint32_t state_activation_time = 0;
  uint32_t active_state_duration = BLE_ACTIVE_STATE_DURATION * 1000;
  bool gps_active;

public:
  T1000eMesh(RADIO_CLASS& phy, RadioLibWrapper& rw, mesh::RNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables, LocationProvider& nmea)
     : BaseCompanionRadioMesh(phy, rw, rng, rtc, tables, board, PUBLIC_GROUP_PSK, LORA_FREQ, LORA_SF, LORA_BW, LORA_CR, LORA_TX_POWER), 
     _nmea(&nmea), _pwm(nRF52_PWM(LED_PIN, 1000.0f, 100.0f)), state{SLEEP}, state_activation_time(millis()), 
     gps_active(false) {

     }

  void ledHandler() {
    static long next_led_update = 0;
    if (millisHasNowPassed(next_led_update)) {
      static bool up;
      static int cycles;
      bool gps_fix = _nmea->isValid();
      bool has_msg = getUnreadMsgNb() > 0;
      int pwm_level;

      if ((++cycles)%2 == 0) {
        pwm_level = (gps_active && gps_fix)? 40 : 0;
        next_led_update = gps_active ? futureMillis(1000) : futureMillis(2000);
      } else {
        if (has_msg) {
          pwm_level = 95;
          next_led_update = futureMillis(200);
        } else if (state == SLEEP) {
          next_led_update = futureMillis(2000);
          pwm_level = 20;
        } else {
          pwm_level = 30;
          next_led_update = futureMillis(1000);
        }
      }
      _pwm.setPWM(LED_PIN, 1000, pwm_level);
    }
  }

  void handleCmdFrame(size_t len) override {
    if (cmd_frame[0] == CMD_CLI_COMMAND && len >= 2) {
      cmd_frame[len] = 0;
      out_frame[0] = RESP_CODE_CLI_RESPONSE;
      handleCommand(0, (char*)&cmd_frame[1], (char *)&out_frame[1]);
      _serial->writeFrame(out_frame, 1 + strlen((char*)&out_frame[1]));
    } else {
      BaseCompanionRadioMesh::handleCmdFrame(len);
    }
    reactivate(); // get more cx time
  }

  void reactivate() {
    if (state == SLEEP) {
      state = ACTIVE;
      //_serial->enable();
      Bluefruit.Advertising.start(0); 
    }
    state_activation_time = millis();
  }

  void deactivate() {
    state = SLEEP;
//    _serial->disable();
    Bluefruit.Advertising.stop();
    state_activation_time = millis();
  }

  void stateHandler() {
    if (state == SLEEP) return;

    static int nextCheck = 0;

    if (millisHasNowPassed(nextCheck)) {
      if (_serial->isConnected()) {
        state_activation_time = millis();
        return;
      }

      if ((active_state_duration > 0)
        && (millis() - state_activation_time > active_state_duration)) {
        deactivate();
      }
      nextCheck = futureMillis(500);
    }
  }

  void startInterface(BaseSerialInterface& serial) override {
    BaseCompanionRadioMesh::startInterface(serial);
    state_activation_time = millis();
    state = ACTIVE;
  }

  void buttonHandler() {
    static int lastBtnState = 0;
    static int btnPressNumber = 0;
    static int lastBtnChangedTime = 0;
    static int nextBtnCheck = 0;

    if (millisHasNowPassed(nextBtnCheck)) {
      int btnState = digitalRead(BUTTON_PIN);
      bool btnChanged = (btnState != lastBtnState);
      if (btnChanged && (btnState == LOW)) { // low = release
        if (millis() > lastBtnChangedTime + 4000) { // poweroff
          _pwm.setPWM(LED_PIN, 0, 0);
          delay(10);
          board.powerOff();
        } else if (millis() > lastBtnChangedTime + 1000) {
          toggleGps();
          reactivate();
        } else { // change state to active for 5 min
          reactivate();
        }
      }

      if (btnChanged) 
        lastBtnChangedTime = millis();

      lastBtnState = btnState;    
      nextBtnCheck = futureMillis(100);  
    }
  }

  void toggleGps() {
    gps_active = !gps_active;
    if (gps_active) {
      _nmea->begin();
    } else {
      _nmea->stop();
    }
  }

  void gpsHandler() {
    static long next_gps_update = 0;

    _nmea->loop();

    if (millisHasNowPassed(next_gps_update)) {
      if (_nmea->isValid()) {
        _prefs.node_lat = ((double)_nmea->getLatitude())/1000000.;
        _prefs.node_lon = ((double)_nmea->getLongitude())/1000000.;
        if (gps_time_sync_needed) {
          getRTCClock()->setCurrentTime(_nmea->getTimestamp());
          gps_time_sync_needed = false;
        }
      }
      next_gps_update = futureMillis(5000);
    }
  }

  void handleCommand(uint32_t timestamp, const char * command, char* reply) {
    while (*command == ' ') command++;   // skip leading spaces

    if (strlen(command) > 4 && command[2] == '|') {  // optional prefix (for companion radio CLI)
      memcpy(reply, command, 3);  // reflect the prefix back
      reply += 3;
      command += 3;
    }

    if (memcmp(command, "hello", 5) == 0) {
      sprintf(reply, "Hey");
    } else { // delegate to base cli
      sprintf(reply, "Unknown command %s", command);
    }
  }

  void loop() {
    BaseCompanionRadioMesh::loop();
    if (gps_active) gpsHandler();
    buttonHandler();
    ledHandler();
    stateHandler();
  }
};

MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1);
T1000eMesh the_mesh(radio, *new WRAPPER_CLASS(radio, board), fast_rng, *new VolatileRTCClock(), tables, nmea);

void halt() {
  while (1) ;
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);

  board.begin();

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio.random(0x7FFFFFFF));
  RadioNoiseListener trng(radio);

  InternalFS.begin();
  the_mesh.begin(InternalFS, trng);

  char dev_name[32+16];
  sprintf(dev_name, "MeshCore-%s", the_mesh.getNodeName());
  serial_interface.begin(dev_name, the_mesh.getBLEPin());

  Bluefruit.setTxPower(-16);    // Check bluefruit.h for supported values

  the_mesh.startInterface(serial_interface);

  // GPS Setup
  nmea.begin();
}

void cli_loop() {
  static char command[80];

  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }
}

void loop() {
  cli_loop();
  the_mesh.loop();
  delay(1); // sync on tick
}