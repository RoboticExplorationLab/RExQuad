#include <LoRa.h>

#include "pose.hpp"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13

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

  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
  if (!LoRa.begin(915E6)) {
    while (1)
      ;
  }
  LoRa.setSpreadingFactor(6);
  LoRa.setSignalBandwidth(500E3);
  LoRa.onReceive(onReceive);
  LoRa.receive(MSG_SIZE);
}

void loop() {
  // int bytes_available = Serial1.available();
  // bool did_receive = bytes_available >= MSG_SIZE;
  // if (did_receive) {
  //   Serial1.readBytes(buf, bytes_available);
  //   Serial.write(buf, MSG_SIZE);
  // }
}