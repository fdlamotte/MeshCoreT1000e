#pragma once

#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <helpers/BaseSerialInterface.h>
#include <RTClib.h>
#include <target.h>


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

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS         100
#endif

#ifndef OFFLINE_QUEUE_SIZE
  #define OFFLINE_QUEUE_SIZE  16
#endif

#ifndef BLE_NAME_PREFIX
  #define BLE_NAME_PREFIX  "MeshCore-"
#endif

#include <helpers/BaseChatMesh.h>
#define SEND_TIMEOUT_BASE_MILLIS          500
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250

/*------------ Frame Protocol --------------*/

#define  PUBLIC_GROUP_PSK  "izOH6cXN6mrJ5e26oRXNcg=="

#define FIRMWARE_VER_CODE    4

#ifndef FIRMWARE_BUILD_DATE
  #define FIRMWARE_BUILD_DATE   "21 Apr 2025"
#endif

#ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION   "v1.5.1_fdl"
#endif

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
#define CMD_SHARE_CONTACT         16
#define CMD_EXPORT_CONTACT        17
#define CMD_IMPORT_CONTACT        18
#define CMD_REBOOT                19
#define CMD_GET_BATTERY_VOLTAGE   20
#define CMD_SET_TUNING_PARAMS     21
#define CMD_DEVICE_QEURY          22
#define CMD_EXPORT_PRIVATE_KEY    23
#define CMD_IMPORT_PRIVATE_KEY    24
#define CMD_SEND_RAW_DATA         25
#define CMD_SEND_LOGIN            26
#define CMD_SEND_STATUS_REQ       27
#define CMD_HAS_CONNECTION        28
#define CMD_LOGOUT                29   // 'Disconnect'
#define CMD_GET_CONTACT_BY_KEY    30
#define CMD_GET_CHANNEL           31
#define CMD_SET_CHANNEL           32
#define CMD_SIGN_START            33
#define CMD_SIGN_DATA             34
#define CMD_SIGN_FINISH           35
#define CMD_SEND_TRACE_PATH       36
#define CMD_SET_DEVICE_PIN        37
#define CMD_SET_OTHER_PARAMS      38

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
#define RESP_CODE_EXPORT_CONTACT   11
#define RESP_CODE_BATTERY_VOLTAGE  12   // a reply to a CMD_GET_BATTERY_VOLTAGE
#define RESP_CODE_DEVICE_INFO      13   // a reply to CMD_DEVICE_QEURY
#define RESP_CODE_PRIVATE_KEY      14   // a reply to CMD_EXPORT_PRIVATE_KEY
#define RESP_CODE_DISABLED         15
#define RESP_CODE_CONTACT_MSG_RECV_V3  16   // a reply to CMD_SYNC_NEXT_MESSAGE (ver >= 3)
#define RESP_CODE_CHANNEL_MSG_RECV_V3  17   // a reply to CMD_SYNC_NEXT_MESSAGE (ver >= 3)
#define RESP_CODE_CHANNEL_INFO     18   // a reply to CMD_GET_CHANNEL
#define RESP_CODE_SIGN_START       19
#define RESP_CODE_SIGNATURE        20

// these are _pushed_ to client app at any time
#define PUSH_CODE_ADVERT            0x80
#define PUSH_CODE_PATH_UPDATED      0x81
#define PUSH_CODE_SEND_CONFIRMED    0x82
#define PUSH_CODE_MSG_WAITING       0x83
#define PUSH_CODE_RAW_DATA          0x84
#define PUSH_CODE_LOGIN_SUCCESS     0x85
#define PUSH_CODE_LOGIN_FAIL        0x86
#define PUSH_CODE_STATUS_RESPONSE   0x87
#define PUSH_CODE_LOG_RX_DATA       0x88
#define PUSH_CODE_TRACE_DATA        0x89
#define PUSH_CODE_NEW_ADVERT        0x8A

#define ERR_CODE_UNSUPPORTED_CMD      1
#define ERR_CODE_NOT_FOUND            2
#define ERR_CODE_TABLE_FULL           3
#define ERR_CODE_BAD_STATE            4
#define ERR_CODE_FILE_IO_ERROR        5
#define ERR_CODE_ILLEGAL_ARG          6

#define MAX_SIGN_DATA_LEN    (8*1024)   // 8K

/* -------------------------------------------------------------------------------------- */

struct NodePrefs {  // persisted to file
  float airtime_factor;
  char node_name[32];
  double node_lat, node_lon;
  float freq;
  uint8_t sf;
  uint8_t cr;
  uint8_t reserved1;
  uint8_t manual_add_contacts;
  float bw;
  uint8_t tx_power_dbm;
  uint8_t unused[3];
  float rx_delay_base;
  uint32_t ble_pin;
};

class BaseCompanionRadioMesh : public BaseChatMesh {
protected:
  FILESYSTEM* _fs;
  IdentityStore* _identity_store;
  uint32_t expected_ack_crc;  // TODO: keep table of expected ACKs
  uint32_t pending_login;
  uint32_t pending_status;
  BaseSerialInterface* _serial;
  unsigned long last_msg_sent;
  ContactsIterator _iter;
  uint32_t _iter_filter_since;
  uint32_t _most_recent_lastmod;
  uint32_t _active_ble_pin;
  bool  _iter_started;
  uint8_t app_target_ver;
  uint8_t* sign_data;
  uint32_t sign_data_len;
  uint8_t cmd_frame[MAX_FRAME_SIZE+1];
  uint8_t out_frame[MAX_FRAME_SIZE+1];
  const char * _psk;

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  int offline_queue_len;
  Frame offline_queue[OFFLINE_QUEUE_SIZE];

  struct AckTableEntry {
    unsigned long msg_sent;
    uint32_t ack;
  };
  #define EXPECTED_ACK_TABLE_SIZE   8
  AckTableEntry  expected_ack_table[EXPECTED_ACK_TABLE_SIZE];  // circular table
  int next_ack_idx;

  void loadMainIdentity() {
    if (!_identity_store->load("_main", self_id)) {
      self_id = radio_new_identity();  // create new random identity
      saveMainIdentity(self_id);
    }
  }

  bool saveMainIdentity(const mesh::LocalIdentity& identity) {
    return _identity_store->save("_main", identity);
  }

  bool isAutoAddEnabled() const override {
    return (_prefs.manual_add_contacts & 1) == 0;
  }

  void loadContacts();
  void saveContacts();
  void loadChannels();
  void saveChannels();
  int  getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[]) override;
  bool putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], int len) override;
  void writeOKFrame();
  void writeErrFrame(uint8_t err_code);
  void writeDisabledFrame();
  void writeContactRespFrame(uint8_t code, const ContactInfo& contact);
  void updateContactFromFrame(ContactInfo& contact, const uint8_t* frame, int len);
  void addToOfflineQueue(const uint8_t frame[], int len);
  int getFromOfflineQueue(uint8_t frame[]);

protected:
  NodePrefs _prefs;
  mesh::MainBoard* _board;

  float getAirtimeBudgetFactor() const override;
  int calcRxDelay(float score, uint32_t air_time) const override;
  void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override;

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    return SEND_TIMEOUT_BASE_MILLIS + 
         ( (pkt_airtime_millis*DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_len + 1));
  }

  void onDiscoveredContact(ContactInfo& contact, bool is_new) override;
  void onContactPathUpdated(const ContactInfo& contact) override;
  bool processAck(const uint8_t *data) override;
  void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override;
  void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t timestamp, const char *text) override;
  void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override;
  void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override;
  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override;
  void onRawDataRecv(mesh::Packet* packet) override;
  void onTraceRecv(mesh::Packet* packet, uint32_t tag, uint32_t auth_code, uint8_t flags, const uint8_t* path_snrs, const uint8_t* path_hashes, uint8_t path_len) override;
  void onSendTimeout() override {}

  virtual void queueMessage(const ContactInfo& from, uint8_t txt_type, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t* extra, int extra_len, const char *text);
  int getUnreadMsgNb() {return offline_queue_len;}
  virtual void onNextMsgSync() {};

  void soundBuzzer();

public:

  BaseCompanionRadioMesh(mesh::Radio& radio, mesh::RNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables, mesh::MainBoard& board, const char* psk, float freq, uint8_t sf, float bw, uint8_t cr, uint8_t tx_power)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables), _serial(NULL), _board(&board), _psk(psk)
  {
    _iter_started = false;
    offline_queue_len = 0;
    app_target_ver = 0;
    _identity_store = NULL;
    pending_login = pending_status = 0;
    next_ack_idx = 0;
    sign_data = NULL;

    // defaults
    memset(&_prefs, 0, sizeof(_prefs));
    _prefs.airtime_factor = 1.0;    // one half
    strcpy(_prefs.node_name, "NONAME");
    _prefs.freq = LORA_FREQ;
    _prefs.sf = LORA_SF;
    _prefs.bw = LORA_BW;
    _prefs.cr = LORA_CR;
    _prefs.tx_power_dbm = LORA_TX_POWER;
    //_prefs.rx_delay_base = 10.0f;  enable once new algo fixed
  }

  virtual void begin(FILESYSTEM& fs);
  const char* getNodeName() { return _prefs.node_name; }
  uint32_t getBLEPin() { return _active_ble_pin; }
  virtual void startInterface(BaseSerialInterface& serial);
  void savePrefs();
  void loadPrefsInt(const char* filename);
  virtual void handleCmdFrame(size_t len);
  void loop();
};
