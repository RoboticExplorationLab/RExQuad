#include <SPI.h>
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

char buf[200];
char msg[MSG_SIZE];
int pos = 0;  // position in buffer

/**
 * Send the data over the LoRa radio
 */
void send_lora(void* buf, int len) {
  LoRa.beginPacket(true);
  LoRa.write((uint8_t*) buf, len);
  LoRa.endPacket();
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(256000);
  Serial1.begin(256000);

  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(6);
  LoRa.setSignalBandwidth(500E3);
  LoRa.enableCrc();
}

void loop() {
  // Read all available bytes into buffer
  int bytes_available = Serial.available();
  int bytes_received;

  // Fast code
  if (bytes_available >= MSG_SIZE) { 
    bytes_received = Serial.readBytes(buf, bytes_available);
    int start_index = 0;
    for (int i = 0; i < MSG_SIZE; ++i) {
      if (buf[i] == MsgID) {
        break;
      }
    }
    memcpy(msg, buf+start_index, MSG_SIZE);
    // Serial1.write(msg, MSG_SIZE);
    send_lora(msg, MSG_SIZE);
  }
  return;
}