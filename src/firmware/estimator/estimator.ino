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
constexpr int kWaitForSerial = 0;
const int kHeartbeatTimeoutMs = 200;
const RXOUTPUT output = MOCAPRATE;
// const RXOUTPUT output = NOOUTPUT;

// Aliases
using Time = uint64_t;
using Pose = rexquad::PoseMsg;
using StateMsg = rexquad::StateMsg;
using StateControl = rexquad::StateControlMsg;

// Constants
constexpr int kMaxBufferSize = 200;
constexpr int kPoseSize = sizeof(Pose) + 1;
constexpr int kStateMsgSize = sizeof(StateMsg) + 1;

// Globals
uint8_t g_bufmocap[kMaxBufferSize];
uint8_t g_bufstate[kStateMsgSize];

Pose g_posemocap;
StateMsg g_statemsg;
rexquad::Heartbeat heartbeat;

// State estimator
rexquad::StateEstimator filter;
rexquad::StateVector xhat;

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
    uint8_t len_mocap = sizeof(g_bufmocap);

    if (rf69.recv(g_bufmocap, &len_mocap)) {
      Time t_mocap_us = curtime_us();
      ++packets_received;
      pose_received = true;
      heartbeat.Pulse();

      // Convert bytes into pose message
      rexquad::PoseFromBytes(g_posemocap, (char*)g_bufmocap);

      // Update State Estimate
      filter.PoseMeasurement(g_posemocap, t_mocap_us);
    }
  }

  // Get Current state estimate
  filter.GetStateEstimate(xhat);

  // Convert to state estimate message and send over serial to Teensy
  //   for now, limit message to the rate of the mocap (100 Hz)
  if (pose_received) {
    rexquad::StateMsgFromVector(g_statemsg, xhat.data());
    rexquad::StateMsgToBytes(g_statemsg, g_bufstate);
    Serial1.write(g_bufstate, kStateMsgSize);
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
        Serial.print(g_posemocap.x, 3);
        Serial.print(", ");
        Serial.print(g_posemocap.y, 3);
        Serial.print(", ");
        Serial.print(g_posemocap.z, 3);
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
        Serial.print(g_posemocap.z, 3);
        Serial.print("  estimate z = ");
        Serial.print(xhat[2], 3);
        Serial.print("\n");
      }
  }
}