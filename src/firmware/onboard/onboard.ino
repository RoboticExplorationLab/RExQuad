#include <SPI.h>
#include <RH_RF95.h>

#include "sensors.hpp"
#include "pose.hpp"
#include "motors.hpp"

// Accelerometer SPI
#define LSM_CS 6 
#define LSM_SCK 15 
#define LSM_MISO 14 
#define LSM_MOSI 16 
rexquad::IMU imu(LSM_CS);

// Motors
#define FRONT_LEFT_PIN 9 
#define FRONT_RIGHT_PIN 10 
#define BACK_RIGHT_PIN 11 
#define BACK_LEFT_PIN 12 
rexquad::QuadMotors motors(FRONT_LEFT_PIN, FRONT_RIGHT_PIN, BACK_RIGHT_PIN, BACK_LEFT_PIN);

// LoRa
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13
#define LED 13

// Options
constexpr bool kWaitForSerial = true;


// Define message payload
using Pose = rexquad::PoseMsg;
constexpr int MSG_SIZE = sizeof(Pose) + 1;
constexpr uint8_t MsgID = Pose::MsgID();
// constexpr int MSG_SIZE = 5; 
// constexpr uint8_t MsgID = 120;

// Radio driver 
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  // Serial setup
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(256000);
  Serial1.begin(256000);

  if (kWaitForSerial) {
    while (!Serial) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  digitalWrite(LED_PIN, HIGH);

  // Accelerometer Setup
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

  // LoRa setup
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  // Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  // rf95.setModemConfig(RH_RF95::Bw500Cr45Sf64);

  // Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  // rf95.set_dragons();
  // rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  // rf95.setModemConfig(RH_RF95::Bw500Cr45Sf64);
}

uint8_t buf[MSG_SIZE];
void loop() {
  imu.ReadSensor();
  imu.PrintData(Serial);
  delay(100);

  // Relay packet
  if (rf95.available()) {
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED, HIGH);
      Serial.write(buf, len);
      digitalWrite(LED, LOW);
    }
  }
}