#include <LoRa.h>
#include <Adafruit_LSM6DSO32.h>

#include "sensors.hpp"
#include "pose.hpp"

// Accelerometer SPI
#define LSM_CS 11 
#define LSM_SCK 15 
#define LSM_MISO 14 
#define LSM_MOSI 16 

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13

// Accelerometer
Adafruit_LSM6DSO32 dso32;
rexquad::IMU imu(LSM_CS);

using Pose = rexquad::PoseMsg;
constexpr int MSG_SIZE = sizeof(Pose) + 1;
constexpr uint8_t MsgID = Pose::MsgID();

char buf[100];

void onReceive(int packetSize) {
  if (packetSize) {
    LoRa.readBytes(buf, MSG_SIZE);
    // Serial.println("LoRa packet received!");
    Serial.write(buf, MSG_SIZE);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(256000);
  Serial1.begin(256000);

  // Accelerometer Setup
  while (!Serial) { delay(10); }
  Serial.println("Sensor test!");
  // if (!dso32.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
  while (!imu.Connect()) {
    Serial.println("Failed to find LSM6DSO32 chip");
    delay(1000);
  }
  Serial.println("Found LSM6DSO32 chip!");

  imu.SetAccelRange(rexquad::IMU::AccelRange::Accel8g);
  // imu.SetGyroRange(rexquad::IMU::GyroRange::Gyro250dps);
  // imu.SetAccelRate(rexquad::IMU::DataRate::Rate12_5Hz);
  // imu.SetGyroRate(rexquad::IMU::DataRate::Rate12_5Hz);

  // Lora Settings
  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
  // if (!LoRa.begin(915E6)) {
  //   while (1)
  //     ;
  // }
  LoRa.setSpreadingFactor(6);
  LoRa.setSignalBandwidth(500E3);
  LoRa.onReceive(onReceive);
  LoRa.receive(MSG_SIZE);
}

void loop() {
  imu.ReadSensor();
  imu.PrintData(Serial);

  delay(100);

  // int bytes_available = Serial1.available();
  // bool did_receive = bytes_available >= MSG_SIZE;
  // if (did_receive) {
  //   Serial1.readBytes(buf, bytes_available);
  //   Serial.write(buf, MSG_SIZE);
  // }
}