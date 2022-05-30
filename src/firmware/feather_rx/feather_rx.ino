#include "constants.hpp"
#include "estimator.hpp"
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
enum RXOUTPUT {
  MOCAPRATE,
  PRINTPOSE,
  NOOUTPUT,
  FILTERRATE,
  ALTCOMP,
};
constexpr int kConnectoToIMU = 0;
constexpr int kWaitForSerial = 1;
const int kHeartbeatTimeoutMs = 200;
// const RXOUTPUT output = MOCAPRATE;
const RXOUTPUT output = NOOUTPUT;

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

// State estimator
rexquad::StateEstimator filter;
rexquad::StateVector xhat;
// rexquad::InputVector u;

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

  // Connect IMU
  if (kConnectoToIMU) {
    bool imu_is_connected = imureal.Connect();
    if (!imu_is_connected) {
      while (1) {
        rexquad::Blink(LED_PIN, 1000, 1);
      }
    }
  }
  if (kWaitForSerial) {
    Serial.println("Connected to IMU!");
  }

  rexquad::InitRadio(rf69, RF69_FREQ, RFM69_RST, LED_PIN, /*encrypt=*/false);

  // TODO: Get the IMU Bias sitting on the ground
  float bias[6];
  memset(bias, 0, sizeof(bias));
  filter.SetBias(bias);

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
  // Process IMU measurement
  imureal.ReadSensor();
  const rexquad::IMUMeasurementMsg& imudata = imureal.GetMeasurement();
  Time t_imu_us = curtime_us();
  filter.IMUMeasurement(imudata, t_imu_us);

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
      filter.PoseMeasurement(pose_mocap, t_mocap_us);
    }
  }

  // Get Current state estimate
  filter.GetStateEstimate(xhat);

  // Convert to state estimate message and send over serial to Teensy
  if (pose_received && packets_received % 25 == 0) {
    rexquad::StateMsgFromVector(g_statemsg, xhat.data());
    rexquad::StateMsgToBytes(g_statemsg, g_bufstate);
    Serial.print("Sent state estimate to Teensy. msgid = ");
    Serial.print(StateMsg::MsgID);
    // Serial.print(" / ");
    // Serial.print(g_bufstate[0]);
    // Serial.print("  payload = [ ");
    // for (int i = 0; i < 3 * sizeof(float) + 1; ++i) {
    //   Serial.print(g_bufstate[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println("]");
    Serial1.write(g_bufstate, kStateMsgSize);
    Serial.print(" position = [ ");
    Serial.print(g_statemsg.x, 3);
    Serial.print(", ");
    Serial.print(g_statemsg.y, 3);
    Serial.print(", ");
    Serial.print(g_statemsg.z, 3);
    Serial.println("]");
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
    case ALTCOMP:
      if (pose_received) {
        Serial.print("received z = ");
        Serial.print(pose_mocap.z, 3);
        Serial.print("  estimate z = ");
        Serial.print(xhat[2], 3);
        Serial.print("\n");
      }
  }
}