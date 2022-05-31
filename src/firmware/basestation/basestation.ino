#include "constants.hpp"
#include "estimator.hpp"
#include "messages.hpp"
#include "pose.hpp"
#include "quad_utils.hpp"
#include "sensors.hpp"
#include "serial_utils.hpp"

// Pin Setup
#define LED_PIN 13

// Radio Setup (Feather Radio)
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4
#define RF69_FREQ 915.0
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Options
enum RXOUTPUT {
  MOCAPRATE,
  PRINTPOSE,
  NOOUTPUT,
};
constexpr int kWaitForSerial = 1;
const int kHeartbeatTimeoutMs = 200;
// const RXOUTPUT output = MOCAPRATE;
const RXOUTPUT output = NOOUTPUT;

// Aliases
using Time = uint64_t;
using StateControl = rexquad::StateControlMsg;

// Constants
constexpr int kMaxBufferSize = 200;
constexpr int kStateControlSize = sizeof(StateControl) + 1;

// Globals
uint8_t g_bufrecv[kMaxBufferSize];
rexquad::StateVector xhat;
rexquad::InputVector u;

StateControl g_statecontrol_msg;
rexquad::Heartbeat heartbeat;

// Timing
Time g_tstart;
Time curtime_us() {
  uint64_t t_micros = micros() - g_tstart;
  return t_micros;
}

/////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
  Serial.begin(256000);
  if (kWaitForSerial) {
    while (!Serial) {
      rexquad::Blink(LED_PIN, 100, 1);
    }
    Serial.println("Connected to Receiver!");
  }
  Serial1.begin(256000);

  // Initialize radio
  rexquad::InitRadio(rf69, RF69_FREQ, RFM69_RST, LED_PIN, /*encrypt=*/false);

  // Setup Heartbeat
  heartbeat.SetTimeoutMs(kHeartbeatTimeoutMs);
  heartbeat.Activate();

  digitalWrite(LED_PIN, LOW);
  Serial.println("Starting loop...");
  g_tstart = micros();  // resets after about 70 minutes per Arduino docs
}

/////////////////////////////////////////////
// Loop
/////////////////////////////////////////////
int packets_received = 0;
void loop() {
  // Process MOCAP pose
  bool pose_received = false;
  if (rf69.available()) {
    uint8_t len_recv = sizeof(g_bufrecv);

    if (rf69.recv(g_bufrecv, &len_recv)) {
      Time t_mocap_us = curtime_us();
      ++packets_received;
      pose_received = true;
      heartbeat.Pulse();

      Serial.print("Got message from Teensy! bytes received = ");
      Serial.println(len_recv);
      // Convert bytes into pose message
      // rexquad::PoseFromBytes(g_statecontrol_msg, (char*)g_bufrecv);
    }
  }

  // Heartbeat indicator
  if (heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }

  // Printing
  switch (output) {
    case MOCAPRATE:
      if (pose_received) {
        rexquad::RatePrinter();
      }
      break;
    case PRINTPOSE:
      if (pose_received) {
        Serial.print("position = [");
        Serial.print(g_statecontrol_msg.x, 3);
        Serial.print(", ");
        Serial.print(g_statecontrol_msg.y, 3);
        Serial.print(", ");
        Serial.print(g_statecontrol_msg.z, 3);
        Serial.println("]");
      }
      break;
    case NOOUTPUT:
      break;
  }
}