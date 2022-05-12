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
}

void QuadMotors::SendPWMCommand(int pwm_fl, int pwm_fr, int pwm_br, int pwm_bl) {
  front_left_esc_.writeMicroseconds(constrain(pwm_fl, kMinInput, kMaxInput));
  front_right_esc_.writeMicroseconds(constrain(pwm_fr, kMinInput, kMaxInput));
  back_right_esc_.writeMicroseconds(constrain(pwm_br, kMinInput, kMaxInput));
  back_left_esc_.writeMicroseconds(constrain(pwm_bl, kMinInput, kMaxInput));
}

void QuadMotors::Arm() {
  front_left_esc_.writeMicroseconds(kMaxInput);
  front_right_esc_.writeMicroseconds(kMaxInput);
  back_right_esc_.writeMicroseconds(kMaxInput);
  back_left_esc_.writeMicroseconds(kMaxInput);
  delay(2000);

  front_left_esc_.writeMicroseconds(kMinInput + 200);
  front_right_esc_.writeMicroseconds(kMinInput + 200);
  back_right_esc_.writeMicroseconds(kMinInput + 200);
  back_left_esc_.writeMicroseconds(kMinInput + 200);
  delay(2000);
}

void QuadMotors::Calibrate() {
  front_left_esc_.writeMicroseconds(kMaxInput);
  front_right_esc_.writeMicroseconds(kMaxInput);
  back_right_esc_.writeMicroseconds(kMaxInput);
  back_left_esc_.writeMicroseconds(kMaxInput);
  delay(7000);

  front_left_esc_.writeMicroseconds(kMinInput);
  front_right_esc_.writeMicroseconds(kMinInput);
  back_right_esc_.writeMicroseconds(kMinInput);
  back_left_esc_.writeMicroseconds(kMinInput);
  delay(8000);
}

void QuadMotors::Kill() {
  front_left_esc_.writeMicroseconds(kMinInput);
  front_right_esc_.writeMicroseconds(kMinInput);
  back_right_esc_.writeMicroseconds(kMinInput);
  back_left_esc_.writeMicroseconds(kMinInput);
  delay(7000);

  front_left_esc_.writeMicroseconds(kMinInput);
  front_right_esc_.writeMicroseconds(kMinInput);
  back_right_esc_.writeMicroseconds(kMinInput);
  back_left_esc_.writeMicroseconds(kMinInput);
  delay(8000);
}

}  // namespace rexquad