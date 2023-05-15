#include <bluefruit.h>
#include "vibe.h" // @TODO - remove this
#include "ble.h"

#define EXPIRE_CPU 100

// max concurrent connections supported by this example
#define MAX_PRPH_CONNECTION 2
static uint8_t connection_count = 0;

struct twig_struct *_twig;

const uint8_t TWIG_UUID_SERVICE[] = {0x14, 0x12, 0x8A, 0x76, 0x04, 0xD1, 0x6C, 0x4F, 0x7E, 0x53, 0xF2, 0xE8, 0x00, 0x00, 0xB1, 0x19};
const uint8_t TWIG_UUID_CHR[] = {0x14, 0x12, 0x8A, 0x76, 0x04, 0xD1, 0x6C, 0x4F, 0x7E, 0x53, 0xF2, 0xE8, 0x01, 0x00, 0xB1, 0x19};

BLEService tbs(TWIG_UUID_SERVICE);
BLECharacteristic tsb(TWIG_UUID_CHR);

void payload_to_struct(struct twig_struct *twig, const uint8_t *payload)
{
  // twig->tap_event = payload[14];
  // twig->tap_event_counter = payload[13];
  // twig->step_count = payload[12];

  twig->yaw = payload[11];
  twig->pitch = payload[10];
  twig->roll = payload[9];

  twig->msg_count = payload[0];
}

// investigate for safety
void tsb_write_callback(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len)
{
  if (len == 15)
  {
    _twig->msg_time = millis();
    _twig->fps++;
    data[9] = 255 - data[9]; // flip reference
    payload_to_struct(_twig, data);
    digitalWrite(LED_RED, _twig->msg_count & 0x01);
  }
}

void connect_check(void)
{
  Serial.print("Connection count: ");
  Serial.println(connection_count);
  if (connection_count < MAX_PRPH_CONNECTION)
  {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
  }
}

void connect_callback(uint16_t conn_handle)
{
  Serial.println(conn_handle);
  Serial.println("Device Connected");
  connection_count++;
  connect_check();
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  Serial.println("Device Disconnected, reason = 0x");
  Serial.println(reason, HEX);
  connection_count--;
  connect_check();
}

void ble_init(struct twig_struct *twig)
{
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  _twig = twig;

  // Initialize Bluefruit with max concurrent connections as Peripheral = MAX_PRPH_CONNECTION, Central = 0
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin(MAX_PRPH_CONNECTION, 0);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  tbs.begin();

  tsb.setProperties(CHR_PROPS_WRITE_WO_RESP);
  tsb.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  tsb.setFixedLen(15);
  tsb.begin();
  tsb.setWriteCallback(tsb_write_callback);

  Serial.println("Setting up the advertising");

  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(tbs);

  Bluefruit.setName("INFINITY TWIG");
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void clear_twig(struct twig_struct *twig)
{
  // cpu->fft[7] = 0;
  // cpu->fft[6] = 0;
  // cpu->fft[5] = 0;
  // cpu->fft[4] = 0;
  // cpu->fft[3] = 0;
  // cpu->fft[2] = 0;
  // cpu->fft[1] = 0;
  // cpu->fft[0] = 0;
}

void expire_twig(void)
{

  if (millis() - _twig->msg_time > EXPIRE_CPU)
    clear_twig(_twig);
}

// @TODO - change this method signature as we need to send
// the quaternion data and not simply booleans i.e. tsb.notify???
// the other option is to use wifi instead of ble so we can send
// larger packets / MTU sizes but we'll have to test performance here
void ble_notify(bool left, bool right)
{
  expire_twig();

  static bool alternate = false; // interleaved

  // Do we want to handle the requests device side?
  // Is this good enough?

  // if (alternate)
  // {
  //   lsbVIBE.notify8(0, vibe_read(left, right));
  // }
  // else
  // {
  //   lsbVIBE.notify8(1, vibe_read(left, right));
  // }
  alternate = !alternate;
}
