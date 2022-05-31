/**
 * @file motor_tests.ino
 * @author Brian Jackson (bjack205@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-18
 * 
 * Allows for motor calibration and for each motor to be tested by running it for
 * half a second
 * 
 * Must be connected to the Serial port on the quadrotor
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

// Radio Wing
#define RF69_FREQ 910.0
#define RFM69_CS 10   // "F"
#define RFM69_INT 9  // "D"
#define RFM69_RST 8  // "C"
RH_RF69 rf69(RFM69_CS, RFM69_INT, hardware_spi);

// Motors
#define FRONT_LEFT_PIN 14 
#define FRONT_RIGHT_PIN 15 
#define BACK_RIGHT_PIN 16 
#define BACK_LEFT_PIN 17 
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

// Timing
uint64_t tstart;
double GetCurrentTimeMs() {
  uint64_t t_micros = micros() - tstart;
  double t_cur = static_cast<double>(t_micros) * 1e-3;
  return t_cur;
}

/////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
  // pinMode(FRONT_LEFT_PIN, OUTPUT);
  // pinMode(FRONT_RIGHT_PIN, OUTPUT);
  // pinMode(BACK_LEFT_PIN, OUTPUT);
  // pinMode(BACK_RIGHT_PIN, OUTPUT);

  // Initialize Serial
  Serial.begin(256000);
  if (kWaitForSerial) {
    while (!Serial) {
      rexquad::Blink(LED_PIN, 100, 1);
    }
  }
  Serial.println("Connected to Onboard Teensy!");

  // Initialize Radio
  rexquad::InitRadio(rf69, RF69_FREQ, RFM69_RST, LED_PIN, /*encrypt=*/false);
  Serial.println("Radio initialized!");

  // Do motor test
  String user_response = rexquad::GetUserResponse(Serial, "Calibrate motors? (y/n)");
  user_response.toLowerCase();
  if (user_response.equals("y")) {
    motors.Calibrate(Serial);
  }
}

enum STATE {
  WAITING_FOR_INPUT,
  RUNNING,
};
char motor = '1';
enum STATE curstate = WAITING_FOR_INPUT;
void loop() {
  using Motor = rexquad::QuadMotors::Motor;
  const int kMinInput = rexquad::kMinInput;
  switch (curstate) {
    case WAITING_FOR_INPUT: {
      String user_response = rexquad::GetUserResponse(Serial, "Select motor (1-4):");
      motor = user_response[0];
      tstart = micros();
      curstate = RUNNING;
      break;
    }
    case RUNNING: {
      switch (motor) {
        case '1':
          motors.SendCommandPWMSingleMotor(Motor::kFrontLeft, kMinInput + 400);
          break;
        case '2':
          motors.SendCommandPWMSingleMotor(Motor::kFrontRight, kMinInput + 400);
          break;
        case '3':
          motors.SendCommandPWMSingleMotor(Motor::kBackRight, kMinInput + 400);
          break;
        case '4':
          motors.SendCommandPWMSingleMotor(Motor::kBackLeft, kMinInput + 400);
          break;
        default:
          motors.SendCommandPWMSingleMotor(Motor::kAllMotors, kMinInput + 400);
      }
      if (GetCurrentTimeMs() > 500) {
        motors.Kill();
        curstate = WAITING_FOR_INPUT;
      }
      break;
    }
  }
}