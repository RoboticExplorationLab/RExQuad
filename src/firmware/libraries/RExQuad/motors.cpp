#include "motors.hpp"

#include <Arduino.h>

#include "constants.hpp"

namespace rexquad {

QuadMotors::QuadMotors(int front_left_pin, int front_right_pin, int back_right_pin,
                       int back_left_pin) {
  front_left_esc_.attach(front_left_pin, kMinInput, kMaxInput);
  front_right_esc_.attach(front_right_pin, kMinInput, kMaxInput);
  back_right_esc_.attach(back_right_pin, kMinInput, kMaxInput);
  back_left_esc_.attach(back_left_pin, kMinInput, kMaxInput);
  SendConstantCommandPWM(kMinInput-100);
}

void QuadMotors::SendCommandPWM(int pwm_fl, int pwm_fr, int pwm_br, int pwm_bl) {
  front_left_esc_.writeMicroseconds(constrain(pwm_fl, kMinInput, kMaxInput));
  front_right_esc_.writeMicroseconds(constrain(pwm_fr, kMinInput, kMaxInput));
  back_right_esc_.writeMicroseconds(constrain(pwm_br, kMinInput, kMaxInput));
  back_left_esc_.writeMicroseconds(constrain(pwm_bl, kMinInput, kMaxInput));
}

void QuadMotors::SendCommandPWMSingleMotor(Motor motor, int pwm) {
  switch (motor) {
    case Motor::kFrontLeft:
      front_left_esc_.writeMicroseconds(constrain(pwm, kMinInput, kMaxInput));
      front_right_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      back_right_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      back_left_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      break;

    case Motor::kFrontRight:
      front_left_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      front_right_esc_.writeMicroseconds(constrain(pwm, kMinInput, kMaxInput));
      back_right_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      back_left_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      break;

    case Motor::kBackRight:
      front_left_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      front_right_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      back_right_esc_.writeMicroseconds(constrain(pwm, kMinInput, kMaxInput));
      back_left_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      break;

    case Motor::kBackLeft:
      front_left_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      front_right_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      back_right_esc_.writeMicroseconds(constrain(0, kMinInput, kMaxInput));
      back_left_esc_.writeMicroseconds(constrain(pwm, kMinInput, kMaxInput));
      break;

    case Motor::kAllMotors:
      SendConstantCommandPWM(pwm);
      break;
  }
}

void QuadMotors::SendConstantCommandPWM(int pwm) {
  SendCommandPWM(pwm, pwm, pwm, pwm);
}

void QuadMotors::Ramp(int cmd_start, int cmd_end, int rate_per_10ms) {
  int dir = 1;
  if (cmd_start > cmd_end) {
    dir = -1;
  }
  for (int cmd = cmd_start; cmd > cmd_end; cmd += rate_per_10ms*dir) {
    SendConstantCommandPWM(cmd);
    delay(10);
  }
}

void QuadMotors::Arm() {
  int mid = (kMaxInput + kMinInput) / 2;
  Ramp(mid, kMinInput - 100, 50);
}

void QuadMotors::Kill() {
  SendConstantCommandPWM(kMinInput-100);
}

void QuadMotors::DeArm() {
  SendConstantCommandPWM(0);
}

void QuadMotors::Stop() {
  SendConstantCommandPWM(kIdleInput);
}

}  // namespace rexquad