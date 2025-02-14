#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/RadioLibWrappers.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <helpers/BaseSerialInterface.h>
#include <RTClib.h>

#include "gps.h"

/* ---------------------------------- CONFIGURATION ------------------------------------- */

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
  #define MAX_LORA_TX_POWER  LORA_TX_POWER
#endif

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS         100
#endif

#ifndef OFFLINE_QUEUE_SIZE
  #define OFFLINE_QUEUE_SIZE  16
#endif

#include <helpers/BaseChatMesh.h>

#define SEND_TIMEOUT_BASE_MILLIS          500
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250

#define  PUBLIC_GROUP_PSK  "izOH6cXN6mrJ5e26oRXNcg=="

#if defined(HELTEC_LORA_V3)
  #include <helpers/HeltecV3Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  static HeltecV3Board board;
#elif defined(HELTEC_LORA_V2)
  #include <helpers/HeltecV2Board.h>
  #include <helpers/CustomSX1276Wrapper.h>
  static HeltecV2Board board;
#elif defined(ARDUINO_XIAO_ESP32C3)
  #include <helpers/XiaoC3Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  #include <helpers/CustomSX1268Wrapper.h>
  static XiaoC3Board board;
#elif defined(SEEED_XIAO_S3) || defined(LILYGO_T3S3)
  #include <helpers/ESP32Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  static ESP32Board board;
#elif defined(RAK_4631)
  #include <helpers/nrf52/RAK4631Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  static RAK4631Board board;
#elif defined(T1000_E)
  #include <helpers/nrf52/T1000eBoard.h>
  #include <helpers/CustomLR1110Wrapper.h>
  static T1000eBoard board;
#else
  #error "need to provide a 'board' object"
#endif

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

/*------------ Frame Protocol --------------*/

#define CMD_APP_START              1
#define CMD_SEND_TXT_MSG           2
#define CMD_SEND_CHANNEL_TXT_MSG   3
#define CMD_GET_CONTACTS           4   // with optional 'since' (for efficient sync)
#define CMD_GET_DEVICE_TIME        5
#define CMD_SET_DEVICE_TIME        6
#define CMD_SEND_SELF_ADVERT       7
#define CMD_SET_ADVERT_NAME        8
#define CMD_ADD_UPDATE_CONTACT     9
#define CMD_SYNC_NEXT_MESSAGE     10
#define CMD_SET_RADIO_PARAMS      11
#define CMD_SET_RADIO_TX_POWER    12
#define CMD_RESET_PATH            13
#define CMD_SET_ADVERT_LATLON     14
#define CMD_REMOVE_CONTACT        15

#define RESP_CODE_OK                0
#define RESP_CODE_ERR               1
#define RESP_CODE_CONTACTS_START    2   // first reply to CMD_GET_CONTACTS
#define RESP_CODE_CONTACT           3   // multiple of these (after CMD_GET_CONTACTS)
#define RESP_CODE_END_OF_CONTACTS   4   // last reply to CMD_GET_CONTACTS
#define RESP_CODE_SELF_INFO         5   // reply to CMD_APP_START
#define RESP_CODE_SENT              6   // reply to CMD_SEND_TXT_MSG
#define RESP_CODE_CONTACT_MSG_RECV  7   // a reply to CMD_SYNC_NEXT_MESSAGE
#define RESP_CODE_CHANNEL_MSG_RECV  8   // a reply to CMD_SYNC_NEXT_MESSAGE
#define RESP_CODE_CURR_TIME         9   // a reply to CMD_GET_DEVICE_TIME
#define RESP_CODE_NO_MORE_MESSAGES 10   // a reply to CMD_SYNC_NEXT_MESSAGE

// these are _pushed_ to client app at any time
#define PUSH_CODE_ADVERT            0x80
#define PUSH_CODE_PATH_UPDATED      0x81
#define PUSH_CODE_SEND_CONFIRMED    0x82
#define PUSH_CODE_MSG_WAITING       0x83

/* -------------------------------------------------------------------------------------- */

struct NodePrefs {  // persisted to file
  float airtime_factor;
  char node_name[32];
  double node_lat, node_lon;
  float freq;
  uint8_t sf;
  uint8_t cr;
  uint8_t reserved1;
  uint8_t reserved2;
  float bw;
  uint8_t tx_power_dbm;
  uint8_t unused[3];
};

class MyMesh : public BaseChatMesh {
  FILESYSTEM* _fs;
  RADIO_CLASS* _phy;
  NodePrefs _prefs;
  uint32_t expected_ack_crc;  // TODO: keep table of expected ACKs
  mesh::GroupChannel* _public;
  BaseSerialInterface* _serial;
  unsigned long last_msg_sent;
  ContactsIterator _iter;
  uint32_t _iter_filter_since;
  uint32_t _most_recent_lastmod;
  bool  _iter_started;
  uint8_t cmd_frame[MAX_FRAME_SIZE+1];
  uint8_t out_frame[MAX_FRAME_SIZE+1];
  bool gps_time_sync_needed = true;

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };
  int offline_queue_len;
  Frame offline_queue[OFFLINE_QUEUE_SIZE];

  void loadContacts() {
    if (_fs->exists("/contacts3")) {
      File file = _fs->open("/contacts3");
      if (file) {
        bool full = false;
        while (!full) {
          ContactInfo c;
          uint8_t pub_key[32];
          uint8_t unused;

          bool success = (file.read(pub_key, 32) == 32);
          success = success && (file.read((uint8_t *) &c.name, 32) == 32);
          success = success && (file.read(&c.type, 1) == 1);
          success = success && (file.read(&c.flags, 1) == 1);
          success = success && (file.read(&unused, 1) == 1);
          success = success && (file.read((uint8_t *) &c.sync_since, 4) == 4);  // was 'reserved'
          success = success && (file.read((uint8_t *) &c.out_path_len, 1) == 1);
          success = success && (file.read((uint8_t *) &c.last_advert_timestamp, 4) == 4);
          success = success && (file.read(c.out_path, 64) == 64);
          success = success && (file.read((uint8_t *) &c.lastmod, 4) == 4);
          success = success && (file.read((uint8_t *) &c.gps_lat, 4) == 4);
          success = success && (file.read((uint8_t *) &c.gps_lon, 4) == 4);

          if (!success) break;  // EOF

          c.id = mesh::Identity(pub_key);
          if (!addContact(c)) full = true;
        }
        file.close();
      }
    }
  }

  void saveContacts() {
#if defined(NRF52_PLATFORM)
    File file = _fs->open("/contacts3", FILE_O_WRITE);
    if (file) { file.seek(0); file.truncate(); }
#else
    File file = _fs->open("/contacts3", "w", true);
#endif
    if (file) {
      ContactsIterator iter;
      ContactInfo c;
      uint8_t unused = 0;

      while (iter.hasNext(this, c)) {
        bool success = (file.write(c.id.pub_key, 32) == 32);
        success = success && (file.write((uint8_t *) &c.name, 32) == 32);
        success = success && (file.write(&c.type, 1) == 1);
        success = success && (file.write(&c.flags, 1) == 1);
        success = success && (file.write(&unused, 1) == 1);
        success = success && (file.write((uint8_t *) &c.sync_since, 4) == 4);
        success = success && (file.write((uint8_t *) &c.out_path_len, 1) == 1);
        success = success && (file.write((uint8_t *) &c.last_advert_timestamp, 4) == 4);
        success = success && (file.write(c.out_path, 64) == 64);
        success = success && (file.write((uint8_t *) &c.lastmod, 4) == 4);
        success = success && (file.write((uint8_t *) &c.gps_lat, 4) == 4);
        success = success && (file.write((uint8_t *) &c.gps_lon, 4) == 4);

        if (!success) break;  // write failed
      }
      file.close();
    }
  }

  void writeOKFrame() {
    uint8_t buf[1];
    buf[0] = RESP_CODE_OK;
    _serial->writeFrame(buf, 1);
  }
  void writeErrFrame() {
    uint8_t buf[1];
    buf[0] = RESP_CODE_ERR;
    _serial->writeFrame(buf, 1);
  }

  void writeContactRespFrame(uint8_t code, const ContactInfo& contact) {
    int i = 0;
    out_frame[i++] = code;
    memcpy(&out_frame[i], contact.id.pub_key, PUB_KEY_SIZE); i += PUB_KEY_SIZE;
    out_frame[i++] = contact.type;
    out_frame[i++] = contact.flags;
    out_frame[i++] = contact.out_path_len;
    memcpy(&out_frame[i], contact.out_path, MAX_PATH_SIZE); i += MAX_PATH_SIZE;
    memcpy(&out_frame[i], contact.name, 32); i += 32;
    memcpy(&out_frame[i], &contact.last_advert_timestamp, 4); i += 4;
    memcpy(&out_frame[i], &contact.gps_lat, 4); i += 4;
    memcpy(&out_frame[i], &contact.gps_lon, 4); i += 4;
    memcpy(&out_frame[i], &contact.lastmod, 4); i += 4;
    _serial->writeFrame(out_frame, i);
  }

  void updateContactFromFrame(ContactInfo& contact, const uint8_t* frame, int len) {
    int i = 0;
    uint8_t code = frame[i++];  // eg. CMD_ADD_UPDATE_CONTACT
    memcpy(contact.id.pub_key, &frame[i], PUB_KEY_SIZE); i += PUB_KEY_SIZE;
    contact.type = frame[i++];
    contact.flags = frame[i++];
    contact.out_path_len = frame[i++];
    memcpy(contact.out_path, &frame[i], MAX_PATH_SIZE); i += MAX_PATH_SIZE;
    memcpy(contact.name, &frame[i], 32); i += 32;
    memcpy(&contact.last_advert_timestamp, &frame[i], 4); i += 4;
    if (i + 8 >= len) {  // optional fields
      memcpy(&contact.gps_lat, &frame[i], 4); i += 4;
      memcpy(&contact.gps_lon, &frame[i], 4); i += 4;
    }
  }

  void addToOfflineQueue(const uint8_t frame[], int len) {
    if (offline_queue_len >= OFFLINE_QUEUE_SIZE) {
      MESH_DEBUG_PRINTLN("ERROR: offline_queue is full!");
    } else {
      offline_queue[offline_queue_len].len = len;
      memcpy(offline_queue[offline_queue_len].buf, frame, len);
      offline_queue_len++;
    }
  }
  int getFromOfflineQueue(uint8_t frame[]) {
    if (offline_queue_len > 0) {   // check offline queue
      size_t len = offline_queue[0].len;   // take from top of queue
      memcpy(frame, offline_queue[0].buf, len);

      offline_queue_len--;
      for (int i = 0; i < offline_queue_len; i++) {   // delete top item from queue
        offline_queue[i] = offline_queue[i + 1];
      }
      return len;
    }
    return 0;  // queue is empty
  }

  void soundBuzzer() {
    // TODO
  }

protected:
  void onDiscoveredContact(ContactInfo& contact, bool is_new) override {
    if (_serial->isConnected()) {
      out_frame[0] = PUSH_CODE_ADVERT;
      memcpy(&out_frame[1], contact.id.pub_key, PUB_KEY_SIZE);
      _serial->writeFrame(out_frame, 1 + PUB_KEY_SIZE);
    } else {
      soundBuzzer();
    }

    saveContacts();
  }

  void onContactPathUpdated(const ContactInfo& contact) override {
    out_frame[0] = PUSH_CODE_PATH_UPDATED;
    memcpy(&out_frame[1], contact.id.pub_key, PUB_KEY_SIZE);
    _serial->writeFrame(out_frame, 1 + PUB_KEY_SIZE);   // NOTE: app may not be connected

    saveContacts();
  }

  bool processAck(const uint8_t *data) override {
    // TODO: see if matches any in a table
    if (memcmp(data, &expected_ack_crc, 4) == 0) {     // got an ACK from recipient
      out_frame[0] = PUSH_CODE_SEND_CONFIRMED;
      memcpy(&out_frame[1], data, 4);
      uint32_t trip_time = _ms->getMillis() - last_msg_sent;
      memcpy(&out_frame[5], &trip_time, 4);
      _serial->writeFrame(out_frame, 9);

      // NOTE: the same ACK can be received multiple times!
      expected_ack_crc = 0;  // reset our expected hash, now that we have received ACK
      return true;
    }
    return false;
  }

  void onMessageRecv(const ContactInfo& from, uint8_t path_len, uint32_t sender_timestamp, const char *text) override {
    int i = 0;
    out_frame[i++] = RESP_CODE_CONTACT_MSG_RECV;
    memcpy(&out_frame[i], from.id.pub_key, 6); i += 6;  // just 6-byte prefix
    out_frame[i++] = path_len;
    out_frame[i++] = TXT_TYPE_PLAIN;
    memcpy(&out_frame[i], &sender_timestamp, 4); i += 4;
    int tlen = strlen(text);   // TODO: UTF-8 ??
    if (i + tlen > MAX_FRAME_SIZE) {
      tlen = MAX_FRAME_SIZE - i;
    }
    memcpy(&out_frame[i], text, tlen); i += tlen;
    addToOfflineQueue(out_frame, i);

    if (_serial->isConnected()) {
      uint8_t frame[1];
      frame[0] = PUSH_CODE_MSG_WAITING;  // send push 'tickle'
      _serial->writeFrame(frame, 1);
    } else {
      soundBuzzer();
    }
  }

  void onChannelMessageRecv(const mesh::GroupChannel& channel, int in_path_len, uint32_t timestamp, const char *text) override {
    int i = 0;
    out_frame[i++] = RESP_CODE_CHANNEL_MSG_RECV;
    out_frame[i++] = 0;  // FUTURE: channel_idx (will just be 'public' for now)
    out_frame[i++] = in_path_len < 0 ? 0xFF : in_path_len;
    out_frame[i++] = TXT_TYPE_PLAIN;
    memcpy(&out_frame[i], &timestamp, 4); i += 4;
    int tlen = strlen(text);   // TODO: UTF-8 ??
    if (i + tlen > MAX_FRAME_SIZE) {
      tlen = MAX_FRAME_SIZE - i;
    }
    memcpy(&out_frame[i], text, tlen); i += tlen;
    addToOfflineQueue(out_frame, i);

    if (_serial->isConnected()) {
      uint8_t frame[1];
      frame[0] = PUSH_CODE_MSG_WAITING;  // send push 'tickle'
      _serial->writeFrame(frame, 1);
    } else {
      soundBuzzer();
    }
  }

  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override {
    // TODO: check for login response
    // TODO: check for Get Stats response
  }

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    return SEND_TIMEOUT_BASE_MILLIS + 
         ( (pkt_airtime_millis*DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_len + 1));
  }

  void onSendTimeout() override {
  }

public:

  MyMesh(RADIO_CLASS& phy, RadioLibWrapper& rw, mesh::RNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : BaseChatMesh(rw, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables), _serial(NULL), _phy(&phy)
  {
    _iter_started = false;
    offline_queue_len = 0;

    // defaults
    memset(&_prefs, 0, sizeof(_prefs));
    _prefs.airtime_factor = 1.0;    // one half
    strcpy(_prefs.node_name, "NONAME");
    _prefs.freq = LORA_FREQ;
    _prefs.sf = LORA_SF;
    _prefs.bw = LORA_BW;
    _prefs.cr = LORA_CR;
    _prefs.tx_power_dbm = LORA_TX_POWER;
  }

  void begin(FILESYSTEM& fs, mesh::RNG& trng) {
    _fs = &fs;

    BaseChatMesh::begin();

  #if defined(NRF52_PLATFORM)
    IdentityStore store(fs, "");
  #else
    IdentityStore store(fs, "/identity");
  #endif
    if (!store.load("_main", self_id)) {
      self_id = mesh::LocalIdentity(&trng);  // create new random identity
      store.save("_main", self_id);
    }

    // load persisted prefs
    if (_fs->exists("/node_prefs")) {
      File file = _fs->open("/node_prefs");
      if (file) {
        file.read((uint8_t *) &_prefs, sizeof(_prefs));
        file.close();
      }
    }

    loadContacts();
    _public = addChannel(PUBLIC_GROUP_PSK); // pre-configure Andy's public channel

    _phy->setFrequency(_prefs.freq);
    _phy->setSpreadingFactor(_prefs.sf);
    _phy->setBandwidth(_prefs.bw);
    _phy->setCodingRate(_prefs.cr);
    _phy->setOutputPower(_prefs.tx_power_dbm);
  }

  const char* getNodeName() { return _prefs.node_name; }

  void startInterface(BaseSerialInterface& serial) {
    _serial = &serial;
    serial.enable();
  }

  void savePrefs() {
#if defined(NRF52_PLATFORM)
    File file = _fs->open("/node_prefs", FILE_O_WRITE);
    if (file) { file.seek(0); file.truncate(); }
#else
    File file = _fs->open("/node_prefs", "w", true);
#endif
    if (file) {
      file.write((const uint8_t *)&_prefs, sizeof(_prefs));
      file.close();
    }
  }

  void handleCmdFrame(size_t len) {
    if (cmd_frame[0] == CMD_APP_START && len >= 8) {   // sent when app establishes connection, respond with node ID
      uint8_t app_ver = cmd_frame[1];
      //  cmd_frame[2..7]  reserved future
      char* app_name = (char *) &cmd_frame[8];
      cmd_frame[len] = 0;  // make app_name null terminated
      MESH_DEBUG_PRINTLN("App %s connected, ver: %d", app_name, (uint32_t)app_ver);

      _iter_started = false;   // stop any left-over ContactsIterator
      int i = 0;
      out_frame[i++] = RESP_CODE_SELF_INFO;
      out_frame[i++] = ADV_TYPE_CHAT;   // what this node Advert identifies as (maybe node's pronouns too?? :-)
      out_frame[i++] = _prefs.tx_power_dbm;
      out_frame[i++] = MAX_LORA_TX_POWER;
      memcpy(&out_frame[i], self_id.pub_key, PUB_KEY_SIZE); i += PUB_KEY_SIZE;

      int32_t lat, lon, alt = 0;
      lat = (_prefs.node_lat * 1000000.0);
      lon = (_prefs.node_lon * 1000000.0);
      memcpy(&out_frame[i], &lat, 4); i += 4;
      memcpy(&out_frame[i], &lon, 4); i += 4;
      memcpy(&out_frame[i], &alt, 4); i += 4;
      
      uint32_t freq = _prefs.freq * 1000;
      memcpy(&out_frame[i], &freq, 4); i += 4;
      uint32_t bw = _prefs.bw*1000;
      memcpy(&out_frame[i], &bw, 4); i += 4;
      out_frame[i++] = _prefs.sf;
      out_frame[i++] = _prefs.cr;

      int tlen = strlen(_prefs.node_name);   // revisit: UTF_8 ??
      memcpy(&out_frame[i], _prefs.node_name, tlen); i += tlen;
      _serial->writeFrame(out_frame, i);
    } else if (cmd_frame[0] == CMD_SEND_TXT_MSG && len >= 14) {
      int i = 1;
      uint8_t txt_type = cmd_frame[i++];
      uint8_t attempt = cmd_frame[i++];
      uint32_t msg_timestamp;
      memcpy(&msg_timestamp, &cmd_frame[i], 4); i += 4;
      uint8_t* pub_key_prefix = &cmd_frame[i]; i += 6;
      ContactInfo* recipient = lookupContactByPubKey(pub_key_prefix, 6);
      if (recipient && attempt < 4 && txt_type == TXT_TYPE_PLAIN) {
        char *text = (char *) &cmd_frame[i];
        int tlen = len - i;
        uint32_t est_timeout;
        text[tlen] = 0;  // ensure null
        int result = sendMessage(*recipient, msg_timestamp, attempt, text, expected_ack_crc, est_timeout);
        // TODO: add expected ACK to table
        if (result == MSG_SEND_FAILED) {
          writeErrFrame();
        } else {
          last_msg_sent = _ms->getMillis();

          out_frame[0] = RESP_CODE_SENT;
          out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
          memcpy(&out_frame[2], &expected_ack_crc, 4);
          memcpy(&out_frame[6], &est_timeout, 4);
          _serial->writeFrame(out_frame, 10);
        }
      } else {
        writeErrFrame(); // unknown recipient, or unsuported TXT_TYPE_*
      }
    } else if (cmd_frame[0] == CMD_SEND_CHANNEL_TXT_MSG) {  // send GroupChannel msg
      int i = 1;
      uint8_t txt_type = cmd_frame[i++];  // should be TXT_TYPE_PLAIN
      uint8_t channel_idx = cmd_frame[i++];   // reserved future
      uint32_t msg_timestamp;
      memcpy(&msg_timestamp, &cmd_frame[i], 4); i += 4;
      const char *text = (char *) &cmd_frame[i];

      if (txt_type == TXT_TYPE_PLAIN && sendGroupMessage(msg_timestamp, *_public, _prefs.node_name, text, len - i)) {   // hard-coded to 'public' channel for now
        writeOKFrame();
      } else {
        writeErrFrame();
      }
    } else if (cmd_frame[0] == CMD_GET_CONTACTS) {  // get Contact list
      if (_iter_started) {
        writeErrFrame();   // iterator is currently busy
      } else {
        if (len >= 5) {   // has optional 'since' param
          memcpy(&_iter_filter_since, &cmd_frame[1], 4);
        } else {
          _iter_filter_since = 0;
        }

        uint8_t reply[5];
        reply[0] = RESP_CODE_CONTACTS_START;
        uint32_t count = getNumContacts();  // total, NOT filtered count
        memcpy(&reply[1], &count, 4);
        _serial->writeFrame(reply, 5);

        // start iterator
        _iter = startContactsIterator();
        _iter_started = true;
        _most_recent_lastmod = 0;
      }
    } else if (cmd_frame[0] == CMD_SET_ADVERT_NAME && len >= 2) {
      int nlen = len - 1;
      if (nlen > sizeof(_prefs.node_name)-1) nlen = sizeof(_prefs.node_name)-1;  // max len
      memcpy(_prefs.node_name, &cmd_frame[1], nlen);
      _prefs.node_name[nlen] = 0; // null terminator
      savePrefs();
      writeOKFrame();
    } else if (cmd_frame[0] == CMD_SET_ADVERT_LATLON && len >= 9) {
      int32_t lat, lon, alt = 0;
      memcpy(&lat, &cmd_frame[1], 4);
      memcpy(&lon, &cmd_frame[5], 4);
      if (len >= 13) {
        memcpy(&alt, &cmd_frame[9], 4);  // for FUTURE support
      }
      if (lat <= 90*1E6 && lat >= -90*1E6 && lon <= 180*1E6 && lon >= -180*1E6) {
        _prefs.node_lat = ((double)lat) / 1000000.0;
        _prefs.node_lon = ((double)lon) / 1000000.0;
        savePrefs();
        writeOKFrame();
      } else {
        writeErrFrame();  // invalid geo coordinate
      }
    } else if (cmd_frame[0] == CMD_GET_DEVICE_TIME) {
      uint8_t reply[5];
      reply[0] = RESP_CODE_CURR_TIME;
      uint32_t now = getRTCClock()->getCurrentTime();
      memcpy(&reply[1], &now, 4);
      _serial->writeFrame(reply, 5);
    } else if (cmd_frame[0] == CMD_SET_DEVICE_TIME && len >= 5) {
      uint32_t secs;
      memcpy(&secs, &cmd_frame[1], 4);
      uint32_t curr = getRTCClock()->getCurrentTime();
      if (secs >= curr) {
        getRTCClock()->setCurrentTime(secs);
        writeOKFrame();
      } else {
        writeErrFrame();
      }
    } else if (cmd_frame[0] == CMD_SEND_SELF_ADVERT) {
      auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
      if (pkt) {
        if (len >= 2 && cmd_frame[1] == 1) {   // optional param (1 = flood, 0 = zero hop)
          sendFlood(pkt);
        } else {
          sendZeroHop(pkt);
        }
        writeOKFrame();
      } else {
        writeErrFrame();
      }
    } else if (cmd_frame[0] == CMD_RESET_PATH && len >= 1+32) {
      uint8_t* pub_key = &cmd_frame[1];
      ContactInfo* recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
      if (recipient) {
        recipient->out_path_len = -1;
        //recipient->lastmod = ??   shouldn't be needed, app already has this version of contact
        saveContacts();
        writeOKFrame();
      } else {
        writeErrFrame();  // unknown contact
      }
    } else if (cmd_frame[0] == CMD_ADD_UPDATE_CONTACT && len >= 1+32+2+1) {
      uint8_t* pub_key = &cmd_frame[1];
      ContactInfo* recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
      if (recipient) {
        updateContactFromFrame(*recipient, cmd_frame, len);
        //recipient->lastmod = ??   shouldn't be needed, app already has this version of contact
        saveContacts();
        writeOKFrame();
      } else {
        ContactInfo contact;
        updateContactFromFrame(contact, cmd_frame, len);
        contact.lastmod = getRTCClock()->getCurrentTime();
        contact.sync_since = 0;
        if (addContact(contact)) {
          saveContacts();
          writeOKFrame();
        } else {
          writeErrFrame();  // table is full!
        }
      }
    } else if (cmd_frame[0] == CMD_REMOVE_CONTACT) {
      uint8_t* pub_key = &cmd_frame[1];
      ContactInfo* recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
      if (recipient && removeContact(*recipient)) {
        saveContacts();
        writeOKFrame();
      } else {
        writeErrFrame();  // not found, or unable to remove
      }
    } else if (cmd_frame[0] == CMD_SYNC_NEXT_MESSAGE) {
      int out_len;
      if ((out_len = getFromOfflineQueue(out_frame)) > 0) {
        _serial->writeFrame(out_frame, out_len);
      } else {
        out_frame[0] = RESP_CODE_NO_MORE_MESSAGES;
        _serial->writeFrame(out_frame, 1);
      }
    } else if (cmd_frame[0] == CMD_SET_RADIO_PARAMS) {
      int i = 1;
      uint32_t freq;
      memcpy(&freq, &cmd_frame[i], 4); i += 4;
      uint32_t bw;
      memcpy(&bw, &cmd_frame[i], 4); i += 4;
      uint8_t sf = cmd_frame[i++];
      uint8_t cr = cmd_frame[i++];

      if (freq >= 300000 && freq <= 2500000 && sf >= 7 && sf <= 12 && cr >= 5 && cr <= 8 && bw >= 7000 && bw <= 500000) {
        _prefs.sf = sf;
        _prefs.cr = cr;
        _prefs.freq = (float)freq / 1000.0;
        _prefs.bw = (float)bw / 1000.0;
        savePrefs();

        _phy->setFrequency(_prefs.freq);
        _phy->setSpreadingFactor(_prefs.sf);
        _phy->setBandwidth(_prefs.bw);
        _phy->setCodingRate(_prefs.cr);
        MESH_DEBUG_PRINTLN("OK: CMD_SET_RADIO_PARAMS: f=%d, bw=%d, sf=%d, cr=%d", freq, bw, (uint32_t)sf, (uint32_t)cr);

        writeOKFrame();
      } else {
        MESH_DEBUG_PRINTLN("Error: CMD_SET_RADIO_PARAMS: f=%d, bw=%d, sf=%d, cr=%d", freq, bw, (uint32_t)sf, (uint32_t)cr);
        writeErrFrame();
      }
    } else if (cmd_frame[0] == CMD_SET_RADIO_TX_POWER) {
      if (cmd_frame[1] > MAX_LORA_TX_POWER) {
        writeErrFrame();
      } else {
        _prefs.tx_power_dbm = cmd_frame[1];
        savePrefs();
        _phy->setOutputPower(_prefs.tx_power_dbm);
        writeOKFrame(); 
      }
    } else {
      writeErrFrame();
      MESH_DEBUG_PRINTLN("ERROR: unknown command: %02X", cmd_frame[0]);
    }
  }

  void loop() {
    BaseChatMesh::loop();
    static long next_gps_update = 0;
    size_t len = _serial->checkRecvFrame(cmd_frame);
    if (len > 0) {
      handleCmdFrame(len);
    } else if (_iter_started    // check if our ContactsIterator is 'running'
      && !_serial->isWriteBusy()    // don't spam the Serial Interface too quickly!
    ) {
      ContactInfo contact;
      if (_iter.hasNext(this, contact)) {
        if (contact.lastmod > _iter_filter_since) {  // apply the 'since' filter
          writeContactRespFrame(RESP_CODE_CONTACT, contact);
          if (contact.lastmod > _most_recent_lastmod) {
            _most_recent_lastmod = contact.lastmod;   // save for the RESP_CODE_END_OF_CONTACTS frame
          }
        }
      } else {  // EOF
        out_frame[0] = RESP_CODE_END_OF_CONTACTS;
        memcpy(&out_frame[1], &_most_recent_lastmod, 4);  // include the most recent lastmod, so app can update their 'since'
        _serial->writeFrame(out_frame, 5);
        _iter_started = false;
      }
    }
    if (millisHasNowPassed(next_gps_update)) {
      if (nmea.isValid()) {
        _prefs.node_lat = ((double)nmea.getLatitude())/1000000.;
        _prefs.node_lon = ((double)nmea.getLongitude())/1000000.;
        if (gps_time_sync_needed) {
          DateTime dt(nmea.getYear(), nmea.getMonth(),nmea.getDay(),nmea.getHour(),nmea.getMinute(),nmea.getSecond());
          getRTCClock()->setCurrentTime(dt.unixtime());
          gps_time_sync_needed = false;
        }
      }
      next_gps_update = futureMillis(5000);
    } 
  }
};

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
MyMesh the_mesh(radio, *new WRAPPER_CLASS(radio, board), fast_rng, *new VolatileRTCClock(), tables);

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

#if defined(NRF52_PLATFORM)
  SPI.setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
  SPI.begin();
#elif defined(P_LORA_SCLK)
  spi.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
#endif
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_TX_POWER, 8, tcxo);
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    halt();
  }

  radio.setCRC(0);

#ifdef SX126X_CURRENT_LIMIT
  radio.setCurrentLimit(SX126X_CURRENT_LIMIT);
#endif

#ifdef SX126X_DIO2_AS_RF_SWITCH
  radio.setDio2AsRfSwitch(SX126X_DIO2_AS_RF_SWITCH);
#endif

  fast_rng.begin(radio.random(0x7FFFFFFF));

  RadioNoiseListener trng(radio);

#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  the_mesh.begin(InternalFS, trng);

#ifdef BLE_PIN_CODE
  char dev_name[32+10];
  sprintf(dev_name, "MeshCore-%s", the_mesh.getNodeName());
  serial_interface.begin(dev_name, BLE_PIN_CODE);
#else
  pinMode(WB_IO2, OUTPUT);
  serial_interface.begin(Serial);
#endif
  the_mesh.startInterface(serial_interface);
#elif defined(ESP32)
  SPIFFS.begin(true);
  the_mesh.begin(SPIFFS, trng);

#ifdef BLE_PIN_CODE
  char dev_name[32+10];
  sprintf(dev_name, "MeshCore-%s", the_mesh.getNodeName());
  serial_interface.begin(dev_name, BLE_PIN_CODE);
#else
  serial_interface.begin(Serial);
#endif
  the_mesh.startInterface(serial_interface);
#else
  #error "need to define filesystem"
#endif

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
