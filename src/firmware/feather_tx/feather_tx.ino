#include "sensors.hpp"
#include "pose.hpp"
#include "quad_utils.hpp"

// Pin Setup
#define LED_PIN 13

// Radio Setup
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4
#define RF69_FREQ 915.0
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Options
const int kHeartbeatTimeoutMs = 1000;

// Constants
constexpr int kMaxBufferSize = 200;
using Pose = rexquad::PoseMsg;
constexpr int kPoseSize = sizeof(Pose) + 1;

// Globals
uint8_t buf[kMaxBufferSize];
uint8_t msg[kPoseSize];
rexquad::Heartbeat heartbeat;

/////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
  Serial.begin(256000);
  rexquad::InitRadio(rf69, RF69_FREQ, RFM69_RST, LED_PIN, /*encrypt=*/false);
  // Serial.println("Connected to Receiver!");

  // Setup Heartbeat
  heartbeat.SetTimeoutMs(kHeartbeatTimeoutMs);
  heartbeat.Activate();

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
    heartbeat.Pulse();
    // Serial.println("Sending Pulse");
    // TODO: Turn LED on/off if it's stale
    // Blink(LED_PIN, 20, 1);
  }
  if (heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
  return;

}