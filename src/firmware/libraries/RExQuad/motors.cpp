#include "motors.hpp"
#include "constants.hpp"

namespace rexquad {

QuadMotors::QuadMotors(int front_left_pin, int front_right_pin, int back_right_pin,
                       int back_left_pin) {
  front_left_esc_.attach(front_left_pin, kMinInput, kMaxInput);
  front_right_esc_.attach(front_right_pin, kMinInput, kMaxInput);
  back_right_esc_.attach(back_right_pin, kMinInput, kMaxInput);
  back_left_esc_.attach(back_left_pin, kMinInput, kMaxInput);
}

// void QuadMotors::SendPWMCommand(int pwm_fl, int pwm_fr, int pwm_br, int pwm_bl) {
// }

// void QuadMotors::Arm() {

// }



}  // namespace rexquad