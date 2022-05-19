#include <RH_RF95.h>
#include <SPI.h>

#include <cstring>

#include "control.hpp"
#include "constants.hpp"
#include "lqr_constants.hpp"
#include "estimator.hpp"
#include "messages.hpp"
#include "motors.hpp"
#include "pose.hpp"
#include "sensors.hpp"

// Options
constexpr bool kWaitForSerial = false;
constexpr bool kIsSim = true;      // is running in simulation environment
constexpr bool kPoseOnly = false;  // only use pose measurements
constexpr bool kGroundTruth = true;  // Use ground truth state estimate
#define EIGEN_NO_MALLOC

// Accelerometer SPI
#define LSM_CS 6
#define LSM_SCK 15
#define LSM_MISO 14
#define LSM_MOSI 16
rexquad::IMU imureal(LSM_CS);
rexquad::IMUSimulated imusim;

// Motors
#define FRONT_LEFT_PIN 9
#define FRONT_RIGHT_PIN 10
#define BACK_RIGHT_PIN 11
#define BACK_LEFT_PIN 12
rexquad::QuadMotors motors(FRONT_LEFT_PIN, FRONT_RIGHT_PIN, BACK_RIGHT_PIN, BACK_LEFT_PIN);

// LoRa
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13
#define LED 13
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Aliases
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
uint8_t
    bufsend[kStateControlSize];  // buffer for sending state estimate and control info back
uint8_t bufrecv[kPoseSize];      // buffer for receiving pose of LoRa
uint8_t bufpose[kPoseSize];

Pose pose_mocap;
StateControl statecontrol;

// State estimator
rexquad::StateEstimator filter;
uint64_t tstart;

// Controller
rexquad::FeedbackGain K;
rexquad::StateVector xhat;  // state estimate
rexquad::InputVector u;
rexquad::ErrorVector e;  // error state
rexquad::StateVector xeq;
rexquad::InputVector ueq;

double curtime() {
  uint64_t t_micros = micros() - tstart;
  double t_cur = static_cast<double>(t_micros) * 1e-6;
  return t_cur;
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

  // Serial setup
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(256000);
  Serial1.begin(256000);

  if (kWaitForSerial) {
    while (!Serial) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  digitalWrite(LED_PIN, HIGH);

  // Accelerometer Setup
  Serial.println("Sensor test!");
  while (!imureal.Connect()) {
    Serial.println("Failed to find LSM6DSO32 chip");
    delay(1000);
  }
  Serial.println("Found LSM6DSO32 chip!");

  imureal.SetAccelRange(rexquad::IMU::AccelRange::Accel8g);
  // imureal.SetGyroRange(rexquad::IMU::GyroRange::Gyro250dps);
  // imureal.SetAccelRate(rexquad::IMU::DataRate::Rate12_5Hz);
  // imureal.SetGyroRate(rexquad::IMU::DataRate::Rate12_5Hz);

  // LoRa setup
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println(
        "Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1)
      ;
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }

  rf95.setTxPower(23, false);
  // rf95.set_dragons();
  // rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  // rf95.setModemConfig(RH_RF95::Bw500Cr45Sf64);

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
  // Process IMU measurement 
  if (!kIsSim) {
    imureal.ReadSensor();
    const rexquad::IMUMeasurementMsg& imudata = imureal.GetMeasurement();
    double t_imu = curtime();
    filter.IMUMeasurement(imudata, t_imu);
  } else {
    // In simulation mode, the IMU data gets sent over serial
    //   before the pose information gets sent over LoRa
    // This reads the imu data sent over serial
    if (!kPoseOnly) {
      Serial.setTimeout(1000);
      imusim.ReadSensor();
      double t_imu = curtime();
      Serial.setTimeout(10);
      const rexquad::IMUMeasurementMsg& imudata = imusim.GetMeasurement();
      filter.IMUMeasurement(imudata, t_imu);
    }
  }

  // Process MOCAP pose 
  bool received_LoRa_packet = false;
  if (rf95.available()) {
    digitalWrite(LED, HIGH);
    uint8_t lenrecv = sizeof(bufrecv);

    if (rf95.recv(bufrecv, &lenrecv)) {
      received_LoRa_packet = true;

      // Convert bytes into pose message
      rexquad::PoseFromBytes(pose_mocap, (char*)bufrecv);
      double t_pose = curtime();

      // TODO: Update state estimate
      filter.PoseMeasurement(pose_mocap, t_pose);
    }
  }

  // Get Current state estimate
  filter.GetStateEstimate(xhat);

  if (kGroundTruth) {
    const rexquad::MeasurementMsg msg = imusim.GetRawMeasurement();
    xhat[0] = msg.x;
    xhat[1] = msg.y;
    xhat[2] = msg.z;
    xhat[3] = msg.qw;
    xhat[4] = msg.qx;
    xhat[5] = msg.qy;
    xhat[6] = msg.qz;
    xhat[7] = msg.vx;
    xhat[8] = msg.vy;
    xhat[9] = msg.vz;
    xhat[10] = msg.wx;
    xhat[11] = msg.wy;
    xhat[12] = msg.wz;
  }

  // TODO: Implement control policy
  rexquad::ErrorState(e, xhat, xeq);
  u = -K * e + ueq;
  for (int i = 0; i < rexquad::kNumInputs; ++i) {
    statecontrol.u[i] = u(i);
  }
  // statecontrol.u[0] = -100;
  // statecontrol.u[1] = -100;
  // statecontrol.u[2] = -100;
  // statecontrol.u[3] = -100;

  // TODO: Send command to motors

  // Send information back over serial
  //   only sends info every time a MOCAP message is received
  if (received_LoRa_packet) {
    if (kIsSim) {
      // Send pose and controls back over serial
      statecontrol.x = xhat[0];
      statecontrol.y = xhat[1];
      statecontrol.z = xhat[2];
      statecontrol.qw = xhat[3];
      statecontrol.qx = xhat[4];
      statecontrol.qy = xhat[5];
      statecontrol.qz = xhat[6];
      statecontrol.vx = xhat[7];
      statecontrol.vy = xhat[8];
      statecontrol.vz = xhat[9];
      statecontrol.wx = xhat[10];
      statecontrol.wy = xhat[11];
      statecontrol.wz = xhat[12];
      rexquad::StateControlMsgToBytes(statecontrol, bufsend);
      uint8_t lensend = sizeof(bufsend);
      Serial.write(bufsend, lensend);
    }
  }
}