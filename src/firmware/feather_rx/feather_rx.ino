#include <RH_RF69.h>

#include "sensors.hpp"
#include "pose.hpp"
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

// Constants
constexpr int kMaxBufferSize = 200;
using Pose = rexquad::PoseMsg;
constexpr int kPoseSize = sizeof(Pose) + 1;

// Globals
uint8_t buf_mocap[kMaxBufferSize];
uint8_t msg[kPoseSize];
Pose pose_mocap;

// Extra functions
void FastBlink() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
}

void Blink(int pin, int delay_ms, int n) {
  for (int i = 0; i < n; ++i) {
    digitalWrite(pin, HIGH);
    delay(delay_ms);
    digitalWrite(pin, LOW);
    delay(delay_ms);
  }
}

/////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
  Serial.begin(256000);
  while (!Serial) {
    Blink(LED_PIN, 100, 1);
  }
  Serial.println("Connected to Receiver!");

  // Connect IMU
  bool imu_is_connected = imureal.Connect();
  if (!imu_is_connected) {
    while (1) {
      Blink(LED_PIN, 1000, 1);
    }
  }
  Serial.println("Connected to IMU!");

  rexquad::InitRadio(rf69, RF69_FREQ, RFM69_RST, LED_PIN, /*encrypt=*/false);
  digitalWrite(LED_PIN, LOW);
}

/////////////////////////////////////////////
// Loop
/////////////////////////////////////////////
void loop() {
  if (rf69.available()) {
    uint8_t len_mocap = sizeof(buf_mocap);

    if (rf69.recv(buf_mocap, &len_mocap)) {
      // Convert bytes into pose message
      rexquad::PoseFromBytes(pose_mocap, (char*)buf_mocap);

      Serial.print("received [");
      Serial.print(len_mocap);
      Serial.print("]: ");
      rexquad::PrintPose(Serial, pose_mocap);
      Blink(LED_PIN, 10, 1);
    }
  }
}