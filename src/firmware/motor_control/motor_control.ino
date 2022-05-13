#include <Servo.h>

#include "motors.hpp"
#include "serial_utils.hpp"

#define MOTOR_PIN 6

#define FRONT_LEFT_PIN 9 
#define FRONT_RIGHT_PIN 10 
#define BACK_RIGHT_PIN 11 
#define BACK_LEFT_PIN 12 

constexpr int kMaxInput = 1832;
constexpr int kMinInput = 1148;


using Motor = rexquad::QuadMotors::Motor;
Motor motor_id = Motor::kFrontLeft;
rexquad::QuadMotors motors(FRONT_LEFT_PIN, FRONT_RIGHT_PIN, BACK_RIGHT_PIN, BACK_LEFT_PIN);

void ramp(int cmd_start, int cmd_end, int cmd_rate) {
  Serial.print("Running Ramp from ");
  Serial.print(cmd_start);
  Serial.print(" to ");
  Serial.println(cmd_end);
  for (int cmd = cmd_start; cmd > cmd_end; cmd -= cmd_rate) {
    Serial.print("Got command ");
    Serial.println(cmd);
    // motor.writeMicroseconds(cmd);
    motors.SendCommandPWMSingleMotor(motor_id, cmd);
    delay(10);
  }
  motors.SendCommandPWMSingleMotor(motor_id, cmd_end);
}

void arm() {
  int mid = (kMaxInput + kMinInput) / 2;
  ramp(mid, kMinInput - 100, 50);
}

void dearm() {
  // motor.writeMicroseconds(0);
  motors.SendCommandPWMSingleMotor(motor_id, 0);
}

void kill() {
  // motor.writeMicroseconds(kMinInput-100);
  motors.SendCommandPWMSingleMotor(motor_id, kMinInput-100);
}

void setup() {
  // put your setup code here, to run once:
  while (!Serial) { delay(10); }
  Serial.begin(57600);
  // motor.attach(MOTOR_PIN);
  // motor.writeMicroseconds(0);
  String user_response;
  user_response = rexquad::GetUserResponse(Serial, "Select motor: ");
  motor_id = static_cast<Motor>(user_response.toInt());

  user_response = rexquad::GetUserResponse(Serial, "Calibrate Motor (power must be off)? (y/n)");
  user_response.toLowerCase();
  if (user_response.equals("y")) {
    motors.SendCommandPWMSingleMotor(motor_id, kMaxInput);
    user_response = rexquad::GetUserResponse(Serial, "Plug in motor. Press any key to continue...");
    motors.SendCommandPWMSingleMotor(motor_id, kMinInput);
    delay(1000);
    Serial.println("Motor calibrated.");
    // motor.writeMicroseconds(kMaxInput);
    // Serial.println("Plug in motors. Press any key to continue..");
    // user_response = Serial.readStringUntil('\n');
    // while (user_response.length() == 0) {
    //   user_response = Serial.readStringUntil('\n');
    // }
    // // ramp(kMaxInput, kMinInput, 70);
    // motor.writeMicroseconds(kMinInput);
    // delay(1000);
    // Serial.println("Motors calibrated.");
  }

}

void loop() {
  if (Serial.available()) {
    String output = Serial.readStringUntil('\n');
    output.trim();
    if (output.equals("a")) {
      Serial.println("Arming");
      arm();
    } else if (output.equals("d")) {
      Serial.println("Dearming");
      dearm();
    } else if (output.equals("k")) {
      Serial.println("Killing");
      kill();
    } else {
      int cmd = output.toInt();
      Serial.print("Got command ");
      Serial.println(output);
      // motor.writeMicroseconds(cmd);
      motors.SendCommandPWMSingleMotor(motor_id, cmd);
    }
    
  }
}