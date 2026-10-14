// Bench test for the MCU UART link (firmware v2.1.0).
// The Arduino only listens on a SoftwareSerial pin and decodes the state frames
// that the MCU sends every 50 ms. It never transmits.
//
// Frame (21 bytes): A5 02 seq flags M1 M2 M3 M4 crc8   (M = int32 LE, crc8 poly 0x07 over bytes 1-19)
// See firmware/README.md section 1.
//
// Wiring (Arduino Uno, 5 V, same level as the MCU):
//   MCU TX (PD1)  -> Arduino pin SOFT_RX_PIN
//   MCU GND       -> Arduino GND
// Serial Monitor: 115200 baud.

#include <SoftwareSerial.h>

const uint8_t  SOFT_RX_PIN = 13;
const uint8_t  SOFT_TX_PIN = 12;   // wired to the MCU RX, but never driven by this sketch
const uint32_t MCU_BAUD = 38400;
const uint32_t PC_BAUD = 115200;
const uint32_t NO_DATA_MS = 2000;

const uint8_t SYNC = 0xA5;
const uint8_t ID_STATE = 0x02;
const uint8_t BODY_LEN = 19;       // seq, flags, 4 x int32, crc

// Wiring order test: listen on both pins in turn (SoftwareSerial listens on one at a time).
SoftwareSerial port_a(SOFT_RX_PIN, SOFT_TX_PIN);
SoftwareSerial port_b(SOFT_TX_PIN, SOFT_RX_PIN);
SoftwareSerial *mcu = &port_a;
uint8_t listen_pin = SOFT_RX_PIN;
const uint32_t SWITCH_MS = 3000;
uint32_t last_switch_ms = 0;

uint8_t body[BODY_LEN];
uint8_t body_pos = 0;
enum State { IDLE, ID, BODY } state = IDLE;

uint32_t ok_count = 0, bad_count = 0, lost_count = 0, byte_count = 0;
uint8_t last_seq = 0;
bool have_seq = false;
uint32_t last_byte_ms = 0, last_report_ms = 0, last_status_ms = 0;

const uint8_t RECENT_LEN = 16;     // raw bytes kept for the "no valid frame yet" line
uint8_t recent[RECENT_LEN];
uint8_t recent_pos = 0;

uint8_t crc8(uint8_t crc, uint8_t b) {
  crc ^= b;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

int32_t read_i32(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

void handle_frame() {
  uint8_t crc = crc8(0, ID_STATE);
  for (uint8_t i = 0; i < BODY_LEN - 1; i++) crc = crc8(crc, body[i]);
  if (crc != body[BODY_LEN - 1]) {
    bad_count++;
    Serial.print(F("BAD CRC: got 0x"));
    Serial.print(body[BODY_LEN - 1], HEX);
    Serial.print(F(" expected 0x"));
    Serial.println(crc, HEX);
    return;
  }
  ok_count++;
  uint8_t seq = body[0];
  if (have_seq) {
    uint8_t gap = (uint8_t)(seq - last_seq - 1);
    if (gap != 0) {
      lost_count += gap;
      Serial.print(F("seq jump, lost "));
      Serial.println(gap);
    }
  }
  last_seq = seq;
  have_seq = true;

  Serial.print(F("seq="));  Serial.print(seq);
  Serial.print(F(" flags=0x")); Serial.print(body[1], HEX);
  Serial.print(F(" M1=")); Serial.print(read_i32(&body[2]));
  Serial.print(F(" M2=")); Serial.print(read_i32(&body[6]));
  Serial.print(F(" M3=")); Serial.print(read_i32(&body[10]));
  Serial.print(F(" M4=")); Serial.print(read_i32(&body[14]));
  Serial.print(F(" | ok=")); Serial.print(ok_count);
  Serial.print(F(" bad=")); Serial.print(bad_count);
  Serial.print(F(" lost=")); Serial.println(lost_count);
}

void feed(uint8_t b) {
  switch (state) {
    case IDLE:
      if (b == SYNC) state = ID;
      break;
    case ID:
      if (b == ID_STATE) { body_pos = 0; state = BODY; }
      else if (b != SYNC) state = IDLE;
      break;
    case BODY:
      body[body_pos++] = b;   // a 0xA5 here is data, never a new frame
      if (body_pos == BODY_LEN) { handle_frame(); state = IDLE; }
      break;
  }
}

void setup() {
  Serial.begin(PC_BAUD);
  port_a.begin(MCU_BAUD);
  port_b.begin(MCU_BAUD);
  // SoftwareSerial made its TX pins outputs. This sketch never sends, and the MCU TX
  // is on one of the two pins, so both must be inputs (no output fight).
  pinMode(SOFT_RX_PIN, INPUT);
  pinMode(SOFT_TX_PIN, INPUT);
  port_a.listen();
  Serial.println(F("MCU UART test: listening at 38400 baud"));
  last_byte_ms = millis();
}

void loop() {
  if (ok_count == 0 && millis() - last_switch_ms > SWITCH_MS) {
    last_switch_ms = millis();
    if (mcu == &port_a) { mcu = &port_b; listen_pin = SOFT_TX_PIN; }
    else                { mcu = &port_a; listen_pin = SOFT_RX_PIN; }
    mcu->listen();
    state = IDLE;
    byte_count = 0;
    Serial.print(F("listening on D"));
    Serial.println(listen_pin);
  }
  while (mcu->available()) {
    uint8_t b = (uint8_t)mcu->read();
    recent[recent_pos++ % RECENT_LEN] = b;
    feed(b);
    byte_count++;
    last_byte_ms = millis();
  }
  if (ok_count == 0 && byte_count > 0 && millis() - last_status_ms > 1000) {
    last_status_ms = millis();
    Serial.print(F("no valid frame yet, bytes="));
    Serial.print(byte_count);
    Serial.print(F(" last bytes:"));
    for (uint8_t i = 0; i < RECENT_LEN; i++) {
      uint8_t v = recent[(recent_pos + i) % RECENT_LEN];
      Serial.print(' ');
      if (v < 16) Serial.print('0');
      Serial.print(v, HEX);
    }
    Serial.println();
  }
  if (millis() - last_byte_ms > NO_DATA_MS && millis() - last_report_ms > NO_DATA_MS) {
    last_report_ms = millis();
    Serial.print(F("no data for 2 s (bytes so far: "));
    Serial.print(byte_count);
    Serial.println(F(")"));
  }
}
