#include "sensors.hpp"
#include "pose.hpp"

// Pin Setup
#define LED_PIN 13

// Radio Setup
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4
#define RF69_FREQ 915.0
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Constants
constexpr int kMaxBufferSize = 200;
using Pose = rexquad::PoseMsg;
constexpr int kPoseSize = sizeof(Pose) + 1;

// Globals
uint8_t buf[kMaxBufferSize];
uint8_t msg[kPoseSize];

// Extra functions
void FastBlink() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
}

void Blink(int pin, int delay_ms, int n) {
  for (int i = 0; i < n; ++i) {
    digitalWrite(pin, HIGH);
    delay(delay_ms);
    digitalWrite(pin, LOW);
    delay(delay_ms);
  }
}

/////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
  Serial.begin(256000);
  rexquad::InitRadio(rf69, RF69_FREQ, RFM69_RST, LED_PIN, /*encrypt=*/false);
  digitalWrite(LED_PIN, LOW);
}

/////////////////////////////////////////////
// Loop 
/////////////////////////////////////////////
int packetnum = 0;
void loop() {
  // Check for data over Serial
  int bytes_available = Serial.available();
  if (bytes_available >= kPoseSize) { 
    int bytes_received = Serial.readBytes((char*)buf, bytes_available);
    int start_index = 0;
    int msgid = Pose::MsgID();
    for (int i = 0; i < bytes_received; ++i) {
      if (buf[i] == msgid) {
        start_index = i;
        break;
      }
    }

    // Send received message over radio 
    memcpy(msg, buf+start_index, kPoseSize);
    rf69.send(msg, kPoseSize);
    rf69.waitPacketSent();
    Blink(LED_PIN, 20, 1);
  }
  return;

}