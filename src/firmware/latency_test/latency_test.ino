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

#include "control.hpp"
#include "motors.hpp"
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

// Motors
#define FRONT_LEFT_PIN 9
#define FRONT_RIGHT_PIN 10
#define BACK_RIGHT_PIN 11
#define BACK_LEFT_PIN 12
rexquad::QuadMotors motors(FRONT_LEFT_PIN, FRONT_RIGHT_PIN, BACK_RIGHT_PIN, BACK_LEFT_PIN);

// Options
constexpr rexquad::QuadMotors::Motor kMotor = rexquad::QuadMotors::Motor::kFrontLeft;
constexpr bool kWaitForSerial = 1;
const float kErrorThreshold = 0.02;
const int kBurnInTimeMs = 500;
const int kTestRunTimeMs = 1000;

// Constants
using Pose = rexquad::PoseMsg;
constexpr int kMaxBufferSize = 200;
constexpr int kPoseSize = sizeof(Pose) + 1;

// Globals
uint8_t buf_mocap[kMaxBufferSize];
uint8_t msg[kPoseSize];
Pose pose_mocap;

// Timing
uint64_t tstart;
double GetCurrentTimeMs() {
  uint64_t t_micros = micros() - tstart;
  double t_cur = static_cast<double>(t_micros) * 1e-3;
  return t_cur;
}

// State machine
enum STATE {
  WAITING_FOR_INPUT,
  BURN_IN,
  RUNNING,
  MOCAP_ECHO,
};
enum STATE curstate = WAITING_FOR_INPUT;
bool initial_state_is_set = false;
bool motion_detected = false;

// Movement detection
void PoseMsgToStateVector(rexquad::StateVector& x, const Pose& pose) {
  x[0] = pose.x;
  x[1] = pose.y;
  x[2] = pose.z;
  x[3] = pose.qw;
  x[4] = pose.qx;
  x[5] = pose.qy;
  x[6] = pose.qz;
}
rexquad::StateVector x0;  // initial state
rexquad::StateVector x;
rexquad::ErrorVector e;  // error state

void MotorOn() {
  motors.SendCommandPWM(1300, 0,0,0);
}
void MotorOff() {
  motors.SendCommandPWM(0,0,0,0);
}

/////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
  // Initialize state vectors
  x0.setZero();
  x.setZero();
  e.setZero();
  x0(3) = 1.0;
  x(3) = 1.0;

  pinMode(FRONT_LEFT_PIN, OUTPUT);
  pinMode(FRONT_RIGHT_PIN, OUTPUT);
  pinMode(BACK_LEFT_PIN, OUTPUT);
  pinMode(BACK_RIGHT_PIN, OUTPUT);

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

  // Do motor test
  Serial.println("Arming motors...");
  motors.Arm();
  Serial.println("Motors Armed!");
  String user_response = rexquad::GetUserResponse(Serial, "Do motor test? (y/n)");
  user_response.toLowerCase();
  if (user_response.equals("y")) {
    Serial.println("Spinning motor...");
    // motors.SendCommandPWMSingleMotor(kMotor, 1400);
    MotorOn();
    delay(500);
    Serial.println("End Test.");
    MotorOff();
    // motors.Kill();
  }

  // // Determine mode
  // user_response = rexquad::GetUserResponse(Serial, "Enter MOCAP echo mode? (y/n)");
  // user_response.toLowerCase();
  // if (user_response.equals("y")) {
  //   curstate = MOCAP_ECHO;
  // } else {
  //   curstate = WAITING_FOR_INPUT;
  // }
  digitalWrite(LED_PIN, LOW);
}

/////////////////////////////////////////////
// Loop
/////////////////////////////////////////////
void loop() {
  uint64_t tcur_ms;
  bool pose_received = false;
  if (rf69.available()) {
    uint8_t len_mocap = sizeof(buf_mocap);

    if (rf69.recv(buf_mocap, &len_mocap)) {
      pose_received = true;

      // Convert bytes into pose message
      rexquad::PoseFromBytes(pose_mocap, (char*)buf_mocap);

      // Serial.print("received [");
      // Serial.print(len_mocap);
      // Serial.print("]: ");
      // rexquad::PrintPose(Serial, pose_mocap);
      // rexquad::Blink(LED_PIN, 10, 1);
      rexquad::print_rate();
    }
  }

  uint64_t t_registered = 0;
  switch (curstate) {
    case WAITING_FOR_INPUT:
      Serial.print("last x0 = [");
      Serial.print(x0(0), 3);
      Serial.print(", ");
      Serial.print(x0(1), 3);
      Serial.print(", ");
      Serial.print(x0(2), 3);
      Serial.print("]\n");
      rexquad::GetUserResponse(Serial, "Press any key to start latency test...");
      tstart = micros();
      initial_state_is_set = false;
      motion_detected = false;
      curstate = BURN_IN;
      MotorOff();
      motors.SendCommandPWMSingleMotor(kMotor, rexquad::kMinInput);
      break;
    case BURN_IN:
      tcur_ms = GetCurrentTimeMs();
      // motors.SendCommandPWMSingleMotor(kMotor, rexquad::kMinInput);
      MotorOff();
      if (!initial_state_is_set && pose_received && (tcur_ms > kBurnInTimeMs / 2)) {
        PoseMsgToStateVector(x0, pose_mocap);
        initial_state_is_set = true;
        Serial.println("Initial state set:");
        rexquad::PrintPose(Serial, pose_mocap);
      }
      if (tcur_ms > kBurnInTimeMs) {
        tstart = micros();
        // motors.SendCommandPWMSingleMotor(kMotor, rexquad::kMinInput+100);
        Serial.println("Sending command!");
        MotorOn();
        curstate = RUNNING;
      }
      break;
    case RUNNING:
      tcur_ms = GetCurrentTimeMs();
      // motors.SendCommandPWMSingleMotor(kMotor, rexquad::kMinInput+100);
      MotorOn();
      if (pose_received) {
        PoseMsgToStateVector(x, pose_mocap);
        rexquad::ErrorState(e, x, x0);
        float err = e.norm();
        if (err > kErrorThreshold && !motion_detected) {
          t_registered = micros() - tstart;
          Serial.print("Registered Command at ");
          Serial.print(t_registered / 1000, 3);
          Serial.println(" ms");
          motion_detected = true;
        }
        if (!motion_detected) {
          Serial.print("Error = ");
          Serial.println(err);
        }
      }
      if (tcur_ms > kTestRunTimeMs) {
        // motors.SendCommandPWMSingleMotor(kMotor, rexquad::kMinInput);
        MotorOff();
        curstate = WAITING_FOR_INPUT;
      }
      break;
    case MOCAP_ECHO:
      MotorOff();
      if (pose_received) {
        rexquad::PrintPose(Serial, pose_mocap);
      }
      break;
  }
}