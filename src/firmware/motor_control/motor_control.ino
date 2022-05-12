#include <Servo.h>

#define MOTOR_PIN 6 
constexpr int kMaxInput = 1832;
constexpr int kMinInput = 1148;

Servo motor;

void ramp(int cmd_start, int cmd_end, int cmd_rate) {
  Serial.print("Running Ramp from ");
  Serial.print(cmd_start);
  Serial.print(" to ");
  Serial.println(cmd_end);
  for (int cmd = cmd_start; cmd > cmd_end; cmd -= cmd_rate) {
    Serial.print("Got command ");
    Serial.println(cmd);
    motor.writeMicroseconds(cmd);
    delay(10);
  }
}

void arm() {
  int mid = (kMaxInput + kMinInput) / 2;
  ramp(mid, kMinInput - 100, 50);
}

void dearm() {
  motor.writeMicroseconds(0);
}

void kill() {
  motor.writeMicroseconds(kMinInput-100);
}

void setup() {
  // put your setup code here, to run once:
  while (!Serial) { delay(10); }
  Serial.begin(57600);
  motor.attach(MOTOR_PIN);
  motor.writeMicroseconds(0);

  Serial.println("Calibrate Motors (power must be off)? (y/n)");
  String user_response = Serial.readStringUntil('\n');
  while (user_response.length() == 0) {
    user_response = Serial.readStringUntil('\n');
  }
  user_response.toLowerCase();
  if (user_response.equals("y")) {
    motor.writeMicroseconds(kMaxInput);
    Serial.println("Plug in motors. Press any key to continue..");
    user_response = Serial.readStringUntil('\n');
    while (user_response.length() == 0) {
      user_response = Serial.readStringUntil('\n');
    }
    ramp(kMaxInput, kMinInput, 10);
    delay(1000);
    Serial.println("Motors calibrated.");
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
      motor.writeMicroseconds(cmd);
    }
    
  }
}