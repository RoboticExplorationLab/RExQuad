/**
 * @file latency_test.ino
 * @author Brian Jackson (bjack205@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-05-17
 *
 * Setup:
 * LoRa connection to MOCAP
 * Connected to motors w/ power
 * Connected via serial to laptop
 *
 * Steps:
 * 1. Test the motor by spinning it for 1 second
 * 2. Wait 1 second before beginning test
 * 3. Start recording the pose and timestep every loop
 * 4. After a delay, send a command to the motor
 * 5. Wait a short time, then stop recordin
 * 6. Send history back over serial
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <RH_RF95.h>
#include <SPI.h>
#include <unistd.h>

#include "constants.hpp"
#include "control.hpp"
#include "estimator.hpp"
#include "lqr_constants.hpp"
#include "messages.hpp"
#include "motors.hpp"
#include "pose.hpp"
#include "sensors.hpp"
#include "serial_utils.hpp"

// Options
constexpr bool kWaitForSerial = true;
const int kBurnInTimeMs = 500;
const int kTestRunTimeMs = 1000;
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
constexpr int kTimeSize = sizeof(double);

// Buffers
uint8_t
    bufsend[kPoseSize + kTimeSize];  // buffer for sending posea and time back over serial
uint8_t bufrecv[kPoseSize];          // buffer for receiving pose of LoRa

// History
Pose pose_mocap;
std::vector<std::pair<double, Pose>> history;

// Timing
uint64_t tstart;
double GetCurrentTimeMs() {
  uint64_t t_micros = micros() - tstart;
  double t_cur = static_cast<double>(t_micros) * 1e-3;
  return t_cur;
}

// Movement detection
rexquad::StateVector x0;  // initial state
rexquad::StateVector x;
rexquad::ErrorVector e;  // error state

// State machine
enum STATE {
  WAITING_FOR_INPUT,
  BURN_IN,
  RUNNING,
  MOCAP_ECHO,
};
enum STATE curstate = WAITING_FOR_INPUT;
bool initial_state_is_set = false;

/////////////////////////////////////////////
// Initialization
/////////////////////////////////////////////
void setup() {
  // Initialize state vectors
  x0.setZero();
  x.setZero();
  e.setZero();

  // Serial setup
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(256000);

  if (kWaitForSerial) {
    while (!Serial) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  Serial.println("Connect to Feather for Latency Test!");

  // Accelerometer Setup
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
  rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  Serial.println("LoRa connected!");

  // Do motor test
  Serial.println("Arming motors...");
  motors.Arm();
  Serial.println("Motors Armed!");
  String user_response = rexquad::GetUserResponse(Serial, "Do motor test? (y/n)");
  user_response.toLowerCase();
  if (user_response.equals("y")) {
    Serial.println("Spinning motor...");
    motors.SendCommandPWMSingleMotor(rexquad::QuadMotors::Motor::kFrontLeft, 1400);
    delay(500);
    Serial.println("End Test.");
    motors.Kill();
  }

  user_response = rexquad::GetUserResponse(Serial, "Enter MOCAP echo mode? (y/n)");
  user_response.toLowerCase();
  if (user_response.equals("y")) {
    curstate = MOCAP_ECHO;
  } else {
    curstate = WAITING_FOR_INPUT;
  }
}

void PrintPose(const Pose& pose) {
  Serial.print("  pos = [");
  Serial.print(pose.x, 3);
  Serial.print(", ");
  Serial.print(pose.y, 3);
  Serial.print(", ");
  Serial.print(pose.z, 3);
  Serial.print("]\n");
  Serial.print("  ori = [");
  Serial.print(pose.qw, 3);
  Serial.print(", ");
  Serial.print(pose.qx, 3);
  Serial.print(", ");
  Serial.print(pose.qy, 3);
  Serial.print(", ");
  Serial.print(pose.qz, 3);
  Serial.print("]\n");
}

void PoseMsgToStateVector(rexquad::StateVector& x, const Pose& pose) {
  x[0] = pose.x;
  x[1] = pose.y;
  x[2] = pose.z;
  x[3] = pose.qw;
  x[4] = pose.qx;
  x[5] = pose.qy;
  x[6] = pose.qz;
}

/////////////////////////////////////////////
// Main Loop
/////////////////////////////////////////////
void loop() {
  double tcur_ms;
  switch (curstate) {
    case WAITING_FOR_INPUT:
      rexquad::GetUserResponse(Serial, "Press any key to start latency test...");
      tstart = micros();  // resets after about 70 minutes per Arduino docs
      initial_state_is_set = false;
      curstate = BURN_IN;
      break;
    case BURN_IN:
      tcur_ms = GetCurrentTimeMs();
      if (tcur_ms > kBurnInTimeMs) {
        motors.SendCommandPWMSingleMotor(rexquad::QuadMotors::Motor::kFrontLeft,
                                         rexquad::kMaxInput);
        curstate = RUNNING;
      }
      break;
    case RUNNING:
      tcur_ms = GetCurrentTimeMs();
      if (tcur_ms > kTestRunTimeMs) {
        motors.Kill();
        curstate = WAITING_FOR_INPUT;
      }
      break;
    case MOCAP_ECHO:
      break;
  }

  if (rf95.available()) {
    digitalWrite(LED, HIGH);
    uint8_t lenrecv = sizeof(bufrecv);

    if (rf95.recv(bufrecv, &lenrecv)) {
      double t_pose = GetCurrentTimeMs();
      // Convert bytes into pose message
      rexquad::PoseFromBytes(pose_mocap, (char*)bufrecv);

      if (curstate == BURN_IN && initial_state_is_set == false) {
        PoseMsgToStateVector(x0, pose_mocap);
        initial_state_is_set = true;
        Serial.println("Initial state set.");
      }

      if (curstate == RUNNING) {
        PoseMsgToStateVector(x, pose_mocap);
        rexquad::ErrorState(e, x, x0);
        float err = e.norm();
        Serial.print("Error = ");
        Serial.println(err);
      }

      if (curstate == MOCAP_ECHO) {
        Serial.println("Got MOCAP packet");
        PrintPose(pose_mocap);
      }
    }
  }
}