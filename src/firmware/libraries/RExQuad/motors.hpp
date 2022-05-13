#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "constants.hpp"
#include "serial_utils.hpp"

namespace rexquad {

class QuadMotors {
 public:
  enum class Motor {
    kAllMotors = -1,
    kFrontLeft = 0,
    kFrontRight = 1,
    kBackRight = 2,
    kBackLeft = 3,
  };

  QuadMotors(int front_left_pin, int front_right_pin, int back_right_pin,
             int back_left_pin);
  void Kill();
  void Arm();
  void DeArm();
  void SendCommandPWM(int pwm_fl, int pwm_fr, int pwm_br, int pwm_bl);
  void SendConstantCommandPWM(int pwm);
  void Ramp(int cmd_start, int cmd_end, int rate_per_10ms);

  /**
   * @brief Calibrates the motors
   * 
   * Must be connected via USB to the serial port.
   * 
   * @tparam SerialClass 
   * @param Serial 
   */
  template <class SerialClass>
  void Calibrate(SerialClass Serial) {
    while (!Serial) { delay(10); }
    String user_response = GetUserResponse(Serial, "Are motors off? (y/n)").toLowerCase();
    bool continue_calibration = false;
    continue_calibration = user_response.equals("y");
    if (!continue_calibration) {
      Serial.println("Exiting calibration");
    }

    SendConstantCommandPWM(kMaxInput);
    user_response = GetUserResponse(Serial, "Plug in motors. Press any key to continue...");
    Ramp(kMaxInput, kMinInput, 10);
    delay(1000);
    Serial.println("Motors calibrated.");
  }

 private:

  Servo front_left_esc_;
  Servo front_right_esc_;
  Servo back_right_esc_;
  Servo back_left_esc_;
};

}  // namespace rexquad