#include <cstring>
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13


// #include "arduino_receiver.hpp"

// rexquad::SerialReceiver<Serial_> receiver(Serial, MsgID);
#include "serial_utils.hpp"
#include "pose.hpp"
using Pose = rexquad::PoseMsg;
constexpr int MSG_SIZE = sizeof(Pose) + 1;
constexpr uint8_t MsgID = Pose::MsgID();

char buf[200];
char msg[MSG_SIZE];
int pos = 0;  // position in buffer

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(256000);
  Serial1.begin(256000);
}

void loop() {
  // Read all available bytes into buffer
  int bytes_available = Serial.available();
  int bytes_received = 0;

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
    Serial1.write(msg, MSG_SIZE);
    // Serial1.write(buf, bytes_received);
  }
  return;

  // Loop over all the read bytes
  // Piece together the message(s) in the `msg` buffer
  // Can send out multiple messages if the received number of bytes is large
  bytes_received = Serial.readBytes(buf, bytes_available);
  for (int i = 0; i < bytes_received; ++i) {
    // // Skip newline characters (for manual testing)
    // if (buf[i] == '\n') {
    //   continue;
    // }

    // If starting a new message, must start with message ID
    if (pos == 0) {
      if (buf[i] == MsgID) {
        msg[pos] = buf[i];
        ++pos;
      }
    // If in the middle of a message, accept all bytes
    } else if (pos < MSG_SIZE) {
      msg[pos] = buf[i];
      ++pos;

      // Message completed
      if (pos == MSG_SIZE) {
        pos = 0;
        Serial.print("Sent a complete message at index ");
        Serial.println(i);
        Serial1.write(msg, MSG_SIZE);
      }
    } 
  }
  if (bytes_received) {
    Serial.print("Position: ");
    Serial.println(pos);
  }

}