#include "constants.hpp"
#include "control.hpp"
#include "estimator.hpp"
#include "lqr_constants.hpp"
#include "messages.hpp"
#include "motors.hpp"
#include "pose.hpp"
#include "quad_utils.hpp"
#include "sensors.hpp"
#include "serial_utils.hpp"

// Options
enum RXOUTPUT { MOCAPRATE, PRINTPOSE, SERIALPOSE, NOOUTPUT, STATEERROR };
constexpr bool kWaitForSerial = 1;
constexpr bool kPrintStateEstimate = 0;
constexpr bool kPrintControl = 0;
const int kHeartbeatTimeoutMs = 100;
// const RXOUTPUT output = NOOUTPUT;
// const RXOUTPUT output = PRINTPOSE;
const RXOUTPUT output = STATEERROR;

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

// Motors
#define FRONT_LEFT_PIN 9
#define FRONT_RIGHT_PIN 10
#define BACK_RIGHT_PIN 11
#define BACK_LEFT_PIN 12
rexquad::QuadMotors motors(FRONT_LEFT_PIN, FRONT_RIGHT_PIN, BACK_RIGHT_PIN, BACK_LEFT_PIN);

// Aliases
using Time = uint64_t;
using Pose = rexquad::PoseMsg;
using Control = rexquad::ControlMsg;
using StateControl = rexquad::StateControlMsg;

// Constants
constexpr int kPoseSize = sizeof(Pose) + 1;
constexpr uint8_t kPoseID = Pose::MsgID();
constexpr int kControlSize = sizeof(Control) + 1;
constexpr uint8_t kControlID = Control::MsgID;
constexpr int kStateControlSize = sizeof(StateControl) + 1;
constexpr uint8_t kStateControlID = StateControl::MsgID;

// Buffers
uint8_t buf_mocap[kPoseSize];  // buffer for receiving MOCAP pose

// Globals
Pose pose_mocap;
StateControl statecontrol;
rexquad::Heartbeat heartbeat;

// State estimator
rexquad::StateEstimator filter;

// Timing
uint64_t tstart;
double GetCurrentTimeMs() {
  uint64_t t_micros = micros() - tstart;
  double t_cur = static_cast<double>(t_micros) * 1e-3;
  return t_cur;
}

// Controller
rexquad::FeedbackGain K;
rexquad::StateVector xhat;  // state estimate
rexquad::InputVector u;
rexquad::ErrorVector e;  // error state
rexquad::StateVector xeq;
rexquad::InputVector ueq;

Time curtime() {
  uint64_t t_micros = micros() - tstart;
  return t_micros;
}

/////////////////////////////////////////////
// Initialization
/////////////////////////////////////////////
void setup() {
  // Initialize LQR controller
  for (int i = 0; i < K.size(); ++i) {
    K(i) = rexquad::kFeedbackGain[i];
  }
  for (int i = 0; i < xeq.size(); ++i) {
    xeq(i) = rexquad::kStateEquilibrium[i];
  }
  for (int i = 0; i < ueq.size(); ++i) {
    ueq(i) = rexquad::kInputEquilibrium[i];
  }

  // Make sure initial commands are 0
  motors.Kill();

  // Initialize Serial
  Serial.begin(256000);
  if (kWaitForSerial) {
    while (!Serial) {
      rexquad::Blink(LED_PIN, 100, 1);
    }
  }
  Serial.println("Connected to Onboard computer!");

  // Connect IMU
  bool imu_is_connected = imureal.Connect();
  if (!imu_is_connected) {
    while (1) {
      rexquad::Blink(LED_PIN, 1000, 1);
    }
  }
  Serial.println("Connected to IMU!");

  // Initialize Radio
  rexquad::InitRadio(rf69, RF69_FREQ, RFM69_RST, LED_PIN, /*encrypt=*/false);
  Serial.println("Radio initialized!");

  // Setup Heartbeat
  heartbeat.SetTimeoutMs(kHeartbeatTimeoutMs);
  heartbeat.Activate();

  // TODO: Get the IMU Bias sitting on the ground
  float bias[6];
  memset(bias, 0, sizeof(bias));
  filter.SetBias(bias);

  // TODO: do a takeoff maneuver and settle into level flight
  tstart = micros();  // resets after about 70 minutes per Arduino docs
}

/////////////////////////////////////////////
// Main Loop
/////////////////////////////////////////////
void loop() {
  imureal.ReadSensor();
  const rexquad::IMUMeasurementMsg& imudata = imureal.GetMeasurement();
  Time t_imu = curtime();
  // filter.IMUMeasurement(imudata, t_imu);

  // Process MOCAP pose
  bool pose_received = false;
  if (rf69.available()) {
    uint8_t len_mocap = sizeof(buf_mocap);

    if (rf69.recv(buf_mocap, &len_mocap)) {
      Time t_mocap = curtime();
      pose_received = true;
      heartbeat.Pulse();

      // Convert bytes into pose message
      rexquad::PoseFromBytes(pose_mocap, (char*)buf_mocap);

      // Update State Estimate
      filter.PoseMeasurement(pose_mocap, t_mocap);
    }
  }

  // Get Current state estimate
  filter.GetStateEstimate(xhat);

  // TODO: Implement control policy
  rexquad::ErrorState(e, xhat, xeq);
  u = -K * e + ueq;

  // Send command only when heartbeat is alive
  if (heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
    motors.Kill();
  } else {
    // Send command to motors
    motors.SendCommandPWM(u(0), u(1), u(2), u(3));
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
    case SERIALPOSE:
      // Serial.write(buf_send, kStateControlSize);
      break;
    case NOOUTPUT:
      break;
    case STATEERROR:
      if (pose_received) {
        // Serial.print("position = [");
        // Serial.print(e(0), 3);
        // Serial.print(", ");
        // Serial.print(e(1), 3);
        // Serial.print(", ");
        // Serial.print(e(2), 3);
        // Serial.println("]");
        Serial.print("attitude = [");
        Serial.print(e(3), 3);
        Serial.print(", ");
        Serial.print(e(4), 3);
        Serial.print(", ");
        Serial.print(e(5), 3);
        Serial.println("]");
      }
  }
  if (kPrintStateEstimate && pose_received) {
    Serial.print("  position = [");
    Serial.print(xhat(0), 3);
    Serial.print(", ");
    Serial.print(xhat(1), 3);
    Serial.print(", ");
    Serial.print(xhat(2), 3);
    Serial.print("]\n");
    Serial.print("  attitude = [");
    Serial.print(xhat(3), 3);
    Serial.print(", ");
    Serial.print(xhat(4), 3);
    Serial.print(", ");
    Serial.print(xhat(5), 3);
    Serial.print(", ");
    Serial.print(xhat(6), 3);
    Serial.print("]\n");
    // Serial.print("  lin vel  = [");
    // Serial.print(xhat(7), 3);
    // Serial.print(", ");
    // Serial.print(xhat(8), 3);
    // Serial.print(", ");
    // Serial.print(xhat(9), 3);
    // Serial.print("]\n");
    // Serial.print("  ang vel  = [");
    // Serial.print(xhat(10), 3);
    // Serial.print(", ");
    // Serial.print(xhat(11), 3);
    // Serial.print(", ");
    // Serial.print(xhat(12), 3);
    // Serial.print("]\n");
  }
  if (kPrintControl && pose_received) {
    Serial.print("  attitude = [");
    Serial.print("  control  = [");
    Serial.print(u(0), 3);
    Serial.print(", ");
    Serial.print(u(1), 3);
    Serial.print(", ");
    Serial.print(u(2), 3);
    Serial.print(", ");
    Serial.print(u(3), 3);
    Serial.print("]\n");
  }
}