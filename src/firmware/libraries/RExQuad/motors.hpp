#pragma once

#include <Servo.h>

#include "constants.hpp"

namespace rexquad {

class QuadMotors {
 public:
  QuadMotors(int front_left_pin, int front_right_pin, int back_right_pin,
             int back_left_pin);
  void Kill();
  void Calibrate();
  void Arm();
  void SendPWMCommand(int pwm_fl, int pwm_fr, int pwm_br, int pwm_bl);

 private:

  Servo front_left_esc_;
  Servo front_right_esc_;
  Servo back_right_esc_;
  Servo back_left_esc_;
};

}  // namespace rexquad