#include "constants.hpp"
#include "messages.hpp"
#include "pose.hpp"
#include "quad_utils.hpp"
#include "sensors.hpp"
#include "serial_utils.hpp"

// Pin Setup
#define LED_PIN 13

// IMU (I2C)
#define LSM_SCL 21
#define LSM_SDA 20
rexquad::IMU imureal;

// Motors
// #define FRONT_LEFT_PIN 9
// #define FRONT_RIGHT_PIN 10  // CONFLICT!
// #define BACK_RIGHT_PIN 11 // CONFLICT!
// #define BACK_LEFT_PIN 12

// Radio Setup
#define RFM69_INT 9   // "D"
#define RFM69_CS 10   // "F" 
#define RFM69_RST 11  // "C"
#define RF69_FREQ 915.0
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// LoRa Wing pinouts
// #define RFM95_INT 9   // "D"
// #define RFM95_CS 10   // "F"
// #define RFM95_RST 11  // "C"
// #define RF95_FREQ 915.0
// RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Options
enum RXOUTPUT {
  MOCAPRATE,
  PRINTPOSE,
  NOOUTPUT,
  FILTERRATE,
};
constexpr int kWaitForSerial = 1;
const int kHeartbeatTimeoutMs = 200;
// const RXOUTPUT output = MOCAPRATE;
const RXOUTPUT output = PRINTPOSE;

// Aliases
using Time = uint64_t;
using Pose = rexquad::PoseMsg;
using StateMsg = rexquad::StateMsg;
using StateControl = rexquad::StateControlMsg;

// Constants
constexpr int kMaxBufferSize = 200;
constexpr int kPoseSize = sizeof(Pose) + 1;
constexpr int kStateControlSize = sizeof(StateControl) + 1;
constexpr int kStateMsgSize = sizeof(StateMsg) + 1;

// Globals
uint8_t buf_mocap[kMaxBufferSize];
uint8_t buf_send[kStateControlSize];
uint8_t g_bufstate[kStateMsgSize];

Pose pose_mocap;
StateControl statecontrol_msg;
StateMsg g_statemsg;
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

  // Initialize radio wing
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
    uint8_t len_mocap = sizeof(buf_mocap);

    if (rf69.recv(buf_mocap, &len_mocap)) {
      Time t_mocap_us = curtime_us();
      ++packets_received;
      pose_received = true;
      heartbeat.Pulse();

      // Convert bytes into pose message
      rexquad::PoseFromBytes(pose_mocap, (char*)buf_mocap);

      // Update State Estimate
      // Serial.println("Got Pose packet!");
      // filter.PoseMeasurement(pose_mocap, t_mocap_us);
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
        Serial.print(pose_mocap.x, 3);
        Serial.print(", ");
        Serial.print(pose_mocap.y, 3);
        Serial.print(", ");
        Serial.print(pose_mocap.z, 3);
        Serial.println("]");
      }
      break;
    case FILTERRATE:
      rexquad::RatePrinter();
    case NOOUTPUT:
      break;
  }
}