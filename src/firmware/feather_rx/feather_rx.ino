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

// Radio Setup
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4
#define RF69_FREQ 915.0
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Options
constexpr int kWaitForSerial = 0;
constexpr int kPrintToSerial = 1;  // Print chars over serial instead of data
const int kHeartbeatTimeoutMs = 1000;

// Constants
using Pose = rexquad::PoseMsg;
using StateControl = rexquad::StateControlMsg;
constexpr int kMaxBufferSize = 200;
constexpr int kPoseSize = sizeof(Pose) + 1;
constexpr int kStateControlSize = sizeof(StateControl) + 1;

// Globals
uint8_t buf_mocap[kMaxBufferSize];
uint8_t buf_send[kStateControlSize];
Pose pose_mocap;
StateControl statecontrol_msg;
rexquad::Heartbeat heartbeat;

rexquad::StateVector xhat;
rexquad::InputVector u;


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

  // Connect IMU
  bool imu_is_connected = imureal.Connect();
  if (!imu_is_connected) {
    while (1) {
      rexquad::Blink(LED_PIN, 1000, 1);
    }
  }
  if (kWaitForSerial) {
    Serial.println("Connected to IMU!");
  }

  rexquad::InitRadio(rf69, RF69_FREQ, RFM69_RST, LED_PIN, /*encrypt=*/false);

  // Setup Heartbeat
  heartbeat.SetTimeoutMs(kHeartbeatTimeoutMs);
  heartbeat.Activate();

  digitalWrite(LED_PIN, LOW);
}

/////////////////////////////////////////////
// Loop
/////////////////////////////////////////////
int packets_received = 0;
void loop() {
  if (rf69.available()) {
    uint8_t len_mocap = sizeof(buf_mocap);

    if (rf69.recv(buf_mocap, &len_mocap)) {
      ++packets_received;
      heartbeat.Pulse();

      // Convert bytes into pose message
      rexquad::PoseFromBytes(pose_mocap, (char*)buf_mocap);

      // Create StateControl Message
      statecontrol_msg.x = pose_mocap.x;
      statecontrol_msg.y = pose_mocap.y;
      statecontrol_msg.z = pose_mocap.z;
      statecontrol_msg.qw = pose_mocap.qw;
      statecontrol_msg.qx = pose_mocap.qx;
      statecontrol_msg.qy = pose_mocap.qy;
      statecontrol_msg.qz = pose_mocap.qz;
      for (int i = 0; i < rexquad::kNumInputs; ++i) {
        u(i) = packets_received;
        statecontrol_msg.u[i] = u(i);
      }
      rexquad::StateControlMsgToBytes(statecontrol_msg, buf_send);
      if (kPrintToSerial) {
        // Serial.print("received [");
        // Serial.print(len_mocap);
        // Serial.print("]: ");

        // Serial.print("position = [");
        // Serial.print(pose_mocap.x, 3);
        // Serial.print(", ");
        // Serial.print(pose_mocap.y, 3);
        // Serial.print(", ");
        // Serial.print(pose_mocap.z, 3);
        // Serial.print("]\n");
        // rexquad::PrintPose(Serial, pose_mocap);
        rexquad::RatePrinter();
      } else {
        Serial.write(buf_send, kStateControlSize);
      }
      // rexquad::Blink(LED_PIN, 10, 1);
    }
  }
  if (heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
  // delay(100);
}