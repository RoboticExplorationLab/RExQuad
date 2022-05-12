#include "motors.hpp"

namespace rexquad {

QuadMotors::QuadMotors(int front_left_pin, int front_right_pin, int back_right_pin,
                       int back_left_pin) {
  front_left_esc_.attach(front_left_pin, MIN_THROTTLE, MAX_THROTTLE);
  front_right_esc_.attach(front_right_pin, MIN_THROTTLE, MAX_THROTTLE);
  back_right_esc_.attach(back_right_pin, MIN_THROTTLE, MAX_THROTTLE);
  back_left_esc_.attach(back_left_pin, MIN_THROTTLE, MAX_THROTTLE);
}

void QuadMotors::SendPWMCommand(int pwm_fl, int pwm_fr, int pwm_br, int pwm_bl) {
}

void QuadMotors::Arm() {

}



}  // namespace rexquad