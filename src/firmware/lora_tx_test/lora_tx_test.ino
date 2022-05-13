// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>
#include "pose.hpp"

// Wing pinouts
#define RFM95_CS 10   // "B"
#define RFM95_RST 11  // "A"
#define RFM95_INT 6 
#define RF95_FREQ 915.0
#define LED_PIN 13

// Feather pinouts
// #define RFM95_CS 8
// #define RFM95_RST 4
// #define RFM95_INT 3

using Pose = rexquad::PoseMsg;
constexpr int MSG_SIZE = sizeof(Pose) + 1;
constexpr uint8_t MsgID = Pose::MsgID();
// constexpr int MSG_SIZE = 5;
// constexpr uint8_t MsgID = 120;

char buf[200];
char msg[MSG_SIZE];
int pos = 0;  // position in buffer

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

/**
 * Send the data over the LoRa radio
 */
void send_lora(void* buf, int len) {
  Serial.println("Sending packet");
  rf95.send((uint8_t *)buf, len);
  rf95.waitPacketSent();
  Serial.println("Packet Sent");
  // LoRa.beginPacket(true);
  // LoRa.write((uint8_t*) buf, len);
  // LoRa.endPacket();
}

void setup() 
{
  // Initialize serial
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(256000);
  Serial1.begin(256000);

  // Setup LoRa
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  delay(100);

  // Serial.println("Feather LoRa TX Test!");

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
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  // rf95.setModemConfig(RH_RF95::Bw500Cr45Sf64);
  
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

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
  // Read all available bytes into buffer
  int bytes_available = Serial.available();
  int bytes_received;

  // Fast code
  if (bytes_available >= MSG_SIZE) { 
    bytes_received = Serial.readBytes(buf, bytes_available);
    int start_index = 0;
    for (int i = 0; i < MSG_SIZE; ++i) {
      if (buf[i] == MsgID) {
        start_index = i;
        break;
      }
    }
    Serial.print("Start index: "); Serial.println(start_index);
    memcpy(msg, buf+start_index, MSG_SIZE);
    // Serial1.write(msg, MSG_SIZE);
    send_lora(msg, MSG_SIZE);
    // Serial.println("Hi");
  }
  return;

  // delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  // Serial.println("Transmitting..."); // Send a message to rf95_server
  
  // char radiopacket[20] = "Hello World #      ";
  // itoa(packetnum++, radiopacket+13, 10);
  // Serial.print("Sending "); Serial.println(radiopacket);
  // radiopacket[19] = 0;
  
  // Serial.println("Sending...");
  // delay(10);
  // rf95.send((uint8_t *)radiopacket, 20);

  // Serial.println("Waiting for packet to complete..."); 
  // delay(10);
  // rf95.waitPacketSent();
  // // Now wait for a reply
  // uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  // uint8_t len = sizeof(buf);

  // Serial.println("Waiting for reply...");
  // if (rf95.waitAvailableTimeout(1000))
  // { 
  //   // Should be a reply message for us now   
  //   if (rf95.recv(buf, &len))
  //  {
  //     Serial.print("Got reply: ");
  //     Serial.println((char*)buf);
  //     Serial.print("RSSI: ");
  //     Serial.println(rf95.lastRssi(), DEC);    
  //   }
  //   else
  //   {
  //     Serial.println("Receive failed");
  //   }
  // }
  // else
  // {
  //   Serial.println("No reply, is there a listener around?");
  // }

}
