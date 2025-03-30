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

#define BATTERY_POINT 12 
static const int Battery_Level_Percent_Table[BATTERY_POINT] = {3200, 3590, 3650, 3700, 3740, 3760, 3795, 3840, 3910, 3980, 4070, 4150};

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
  bool _repeat_en = false;
  int8_t ble_tx;
 
  unsigned int last_uptime = 0;
  unsigned int activation_time;

public:
  T1000eMesh(mesh::Radio& radio, mesh::RNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables, LocationProvider& nmea)
     : BaseCompanionRadioMesh(radio, rng, rtc, tables, board, PUBLIC_GROUP_PSK, LORA_FREQ, LORA_SF, LORA_BW, LORA_CR, LORA_TX_POWER), 
     _nmea(&nmea), _pwm(nRF52_PWM(LED_PIN, 1000.0f, 100.0f)), state{SLEEP}, state_activation_time(millis()), 
      gps_active(false) {
      activation_time = millis();
      ble_tx = Bluefruit.getTxPower();
  }

  void loadT1000Prefs() {
    File file = _fs->open("/t1000_prefs");
    if (file) {
      int8_t val;
      uint32_t uintval;
      int ret;
      if (ret = file.read(&uintval, sizeof(uintval))) 
        active_state_duration = uintval;
      if (ret = file.read(&val, 1)) gps_active = (val != 0);
      if (ret = file.read(&val, 1)) {
        ble_tx = val;
        Bluefruit.setTxPower(ble_tx);
      }
      file.close();
    }
  }

  void saveT1000Prefs() {
    File file = _fs->open("/t1000_prefs", FILE_O_WRITE);
    if (file) { 
      file.seek(0); 
      file.truncate(); 
      file.write((uint8_t*)&active_state_duration, sizeof(active_state_duration));

      uint8_t gpval = gps_active ? 1 : 0;
      file.write(&gpval, 1);
      file.write((uint8_t*)&ble_tx, 1);
      file.close();
    }
  }

  bool allowPacketForward(const mesh::Packet* packet) override {
    return _repeat_en;
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

  void start_gps() {
    gps_active = true;
    //_nmea->begin();
    // this init sequence should be better 
    // comes from seeed examples and deals with all gps pins
    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, HIGH);
    delay(10);
    pinMode(GPS_VRTC_EN, OUTPUT);
    digitalWrite(GPS_VRTC_EN, HIGH);
    delay(10);
         
    pinMode(GPS_RESET, OUTPUT);
    digitalWrite(GPS_RESET, HIGH);
    delay(10);
    digitalWrite(GPS_RESET, LOW);
         
    pinMode(GPS_SLEEP_INT, OUTPUT);
    digitalWrite(GPS_SLEEP_INT, HIGH);
    pinMode(GPS_RTC_INT, OUTPUT);
    digitalWrite(GPS_RTC_INT, LOW);
    pinMode(GPS_RESETB, INPUT_PULLUP);
  }

  void sleep_gps() {
    digitalWrite(GPS_VRTC_EN, HIGH);
    digitalWrite(GPS_EN, LOW);
    digitalWrite(GPS_RESET, HIGH);
    digitalWrite(GPS_SLEEP_INT, HIGH);
    digitalWrite(GPS_RTC_INT, LOW);
    pinMode(GPS_RESETB, OUTPUT);
    digitalWrite(GPS_RESETB, LOW);
    //_nmea->stop();
  }

  void stop_gps() {
    digitalWrite(GPS_VRTC_EN, LOW);
    digitalWrite(GPS_EN, LOW);
    digitalWrite(GPS_RESET, HIGH);
    digitalWrite(GPS_SLEEP_INT, HIGH);
    digitalWrite(GPS_RTC_INT, LOW);
    pinMode(GPS_RESETB, OUTPUT);
    digitalWrite(GPS_RESETB, LOW);
    //_nmea->stop();
  }

  void toggleGps() {
    gps_active = !gps_active;
    if (gps_active) {
      start_gps();
    } else {
      stop_gps();
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

  // function from seeed t1000 examples ...
  uint8_t bat_vol_to_percentage(uint16_t voltage) {                  
      if (voltage < Battery_Level_Percent_Table[0]) {              
          return 0;  
      }

      if (voltage < Battery_Level_Percent_Table[1]) {              
          return 0 + (20UL * (int)(voltage - Battery_Level_Percent_Table[0])) /
                         (int)(Battery_Level_Percent_Table[1] - Battery_Level_Percent_Table[0]);
      }

      for (uint8_t i = 0; i < BATTERY_POINT; i++) {              
          if (voltage < Battery_Level_Percent_Table[i]) {          
              return 20 + (8 * (i - 2)) + (8UL * (int)(voltage - Battery_Level_Percent_Table[i - 1])) / (int)(Battery_Level_Percent_Table[i] - Battery_Level_Percent_Table[i - 1]);
          }   
      }
                     
      return 100;    
  }

  void handleCommand(uint32_t timestamp, const char * command, char* reply) {
    while (*command == ' ') command++;   // skip leading spaces

    if (strlen(command) > 4 && command[2] == '|') {  // optional prefix (for companion radio CLI)
      memcpy(reply, command, 3);  // reflect the prefix back
      reply += 3;
      command += 3;
    }

    if (memcmp(command, "advert", 6) == 0) {
      auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
      sendZeroHop(pkt);
      strcpy(reply, "OK - Advert sent");
    } else if (memcmp(command, "floodadv", 8) == 0) {
      auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
      sendFlood(pkt);
      strcpy(reply, "OK - Flood Advert sent");
    } else if (memcmp(command, "uptime", 6) == 0) {
      unsigned int uptime = (millis() - activation_time) / 1000;
      sprintf(reply, "uptime : %dh %dm %ds, last : %dh %dm",
      uptime / 3600, (uptime % 3600) / 60, uptime % 60,
      last_uptime / 3600, (last_uptime % 3600) / 60);
    } else if (memcmp(command, "bat", 3) == 0) {
      int v = _board->getBattMilliVolts();
      int p = bat_vol_to_percentage(v);
      sprintf(reply, "Bat: %0.2fV/%d%%", ((double)v)/1000., p);
    } else if (memcmp(command, "pinval ", 7) == 0) {
      sprintf(reply, "value : %s", digitalRead(atoi(&command[7])) == HIGH ? "HIGH" : "LOW");
    } else if (memcmp(command, "gps_sync", 8) == 0) {
      gps_time_sync_needed = true;
      if (gps_active) {
        strcpy(reply, "ok");
      } else {
        strcpy(reply, "Don't forget to activate gps");
      }
    } else if (memcmp(command, "reboot", 6) == 0) {
      _board->reboot();
    } else if (memcmp(command, "set ", 4) == 0) {
      const char* config = &command[4];
      if (memcmp(config, "blesleep ", 9) == 0) {
        active_state_duration = 1000 * atoi(&config[9]);
        strcpy(reply, "ok");
        saveT1000Prefs();
      } else if (memcmp(config, "pin ", 4) == 0) {
        _prefs.ble_pin = atoi(&config[4]);
        savePrefs();
        sprintf(reply, "ble pin set to %d", _prefs.ble_pin);
      } else if (memcmp(config, "gps ", 4) == 0) {
        if (memcmp(&config[4], "on", 2) == 0) {
          gps_active = true;
          start_gps();
          strcpy(reply, "gps on");
        } else {
          gps_active = false;
          stop_gps();
          strcpy(reply, "gps off");
        }
        saveT1000Prefs();
      } else if (memcmp(config, "repeat ", 7) == 0) {
        if (memcmp(&config[7], "on", 2) == 0) {
          _repeat_en = true;
          strcpy(reply, "repeat on");
        } else {
          _repeat_en = false;
          strcpy(reply, "repeat off");
        }
      } else if (memcmp(config, "ble_tx ", 7) == 0) {
        Bluefruit.setTxPower(atoi(&config[7]));
        ble_tx = Bluefruit.getTxPower();
        sprintf(reply, "ble tx power %d", ble_tx);
        saveT1000Prefs();
      }
    } else if (memcmp(command, "get ", 4) == 0) {
      const char* config = &command[4];
      if (memcmp(config, "blesleep", 9) == 0) {
        sprintf(reply, "blesleep = %d", active_state_duration / 1000);
      } else if (memcmp(config, "gps", 3) == 0) {
        sprintf(reply, "gps %s", gps_active ? "on" : "off");
      } else if (memcmp(config, "repeat", 6) == 0) {
        sprintf(reply, "repeat %s", _repeat_en ? "on" : "off");
      } else if (memcmp(config, "pin", 3) == 0) {
        sprintf(reply, "prefs ble pin %d, active ble pin %d", _prefs.ble_pin, _active_ble_pin);
      } else if (memcmp(config, "ble_tx", 6) == 0) {
        sprintf(reply, "ble tx power %d", Bluefruit.getTxPower());
      }
    } else { // delegate to base cli
      sprintf(reply, "Unknown command %s", command);
    }
  }

  void begin(FILESYSTEM& fs) override {
    BaseCompanionRadioMesh::begin(fs);

    loadT1000Prefs();

    if (_fs->exists("uptime")) {
      _fs->rename("uptime", "last_uptime");
      File file = _fs->open("last_uptime");
      if (file) {
        file.read((uint8_t*) &last_uptime, sizeof(last_uptime));
        file.close();
      }
    }

    if (gps_active) {
//      _nmea->begin();
      start_gps();
    } else {
      sleep_gps(); // let gps vrtc active
    }
  }

  void loop() {
    BaseCompanionRadioMesh::loop();
    if (gps_active) gpsHandler();
    buttonHandler();
    ledHandler();
    stateHandler();
    
    static int next_uptime_write = 0;
    if (millisHasNowPassed(next_uptime_write)) {
      unsigned int uptime = (millis() - activation_time) / 1000;
      File file = _fs->open("uptime", FILE_O_WRITE);
      if (file) { 
        file.seek(0); 
        file.truncate();
        file.write((uint8_t*) &uptime, sizeof(uptime));
        file.close();
      }
      
      next_uptime_write = futureMillis(60*1000); // every minute
    }
  }
};

MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1);
T1000eMesh the_mesh(radio_driver, fast_rng, *new VolatileRTCClock(), tables, nmea);

void halt() {
  while (1) ;
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);

  board.begin();

  // make sure gps pin are off
  digitalWrite(GPS_VRTC_EN, LOW);
  digitalWrite(GPS_RESET, LOW);
  digitalWrite(GPS_SLEEP_INT, LOW);
  digitalWrite(GPS_RTC_INT, LOW);
  pinMode(GPS_RESETB, OUTPUT);
  digitalWrite(GPS_RESETB, LOW);

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_get_rng_seed());

  InternalFS.begin();
  the_mesh.begin(InternalFS);

  char dev_name[32+16];
  sprintf(dev_name, "MeshCore-%s", the_mesh.getNodeName());
  serial_interface.begin(dev_name, the_mesh.getBLEPin());

//  Bluefruit.setTxPower(-16);    // Check bluefruit.h for supported values

  the_mesh.startInterface(serial_interface);

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