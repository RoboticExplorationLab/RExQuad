#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13

#include "pose.hpp"

using Pose = rexquad::PoseMsg;
constexpr int MSG_SIZE = sizeof(Pose) + 1;
constexpr uint8_t MsgID = Pose::MsgID();

char buf[100];

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(256000);
  Serial1.begin(256000);
}

void loop() {
  // bool did_receive = receiver.Receive(buf, MSG_SIZE);
  int bytes_available = Serial1.available();
  bool did_receive = bytes_available >= MSG_SIZE;
  if (did_receive) {
    Serial1.readBytes(buf, bytes_available);
    Serial.write(buf, MSG_SIZE);
    // Serial.write(buf, bytes_read);
  }
}