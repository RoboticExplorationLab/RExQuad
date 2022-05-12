/**
 * @file lora_relay.ino
 * @author Brian Jackson (bjack205@gmail.com)
 * @brief This script takes input from a computer (e.g. Jetson Nano) and publishes the data
 *        over LoRa radio.
 *
 * Board: LoRa Feather M0 by Adafruit
 *
 * The data is assumed to be rexquad::Pose<int16_t> message, a 24 byte message containing
 * position and orientation information. The microcontroller will send periodic messages
 * back over serial verifying the LoRa is actually sending data.
 *
 * The LED light on the Feather is on whenever data is being transmited. If it stops
 * receiving data, it will wait for 5 seconds for more data before turning off the light.
 *
 * This script uses the message ID to improve robustness when receiving serial data from
 * the computer. The message is assumed to always be the same length.
 *
 * @version 0.1
 * @date 2021-09-24
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <string>

#include <SPI.h>
#include <LoRa.h>

#include "pose.hpp"
#include "arduino_receiver.hpp"
#include "serial_utils.hpp"

// using Pose = rexquad::Pose<int16_t>;
using Pose = rexquad::PoseMsg;

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13
constexpr int MSG_SIZE = sizeof(Pose);

// Global variables
bool has_sent = false;
int time_start = 0;
int count = 0;
char buf[MSG_SIZE];
bool is_stale = true;
int seconds_to_stale = 5;
int timestamp_last_receive_ms = 2 * seconds_to_stale * 1000; // starts as stale
int ison = false;

int tblink_start = 0;

// Initialize objects
rexquad::PrintAtRate stale_printer(1.0);
rexquad::PrintAtRate sending_printer(1.0 / 0.5);  // print a status every 10 seconds
rexquad::SerialReceiver<Serial_> receiver(Serial, Pose::MsgID());


void setup() {
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  Serial.begin(57600);
  // while (!Serial) { delay(10); }

  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(6);
  LoRa.setSignalBandwidth(500E3);
  LoRa.enableCrc();

  digitalWrite(LED_PIN, LOW);
  time_start = micros();
  tblink_start = millis();
}

/**
 * Send the data over the LoRa radio
 */
void send_buf() {
  LoRa.beginPacket(true);
  LoRa.write((uint8_t*)buf, MSG_SIZE);
  LoRa.endPacket();
}


void loop() {
  int telapsed = millis() - tblink_start;

  // Receive data from the serial port
  bool did_receive = receiver.Receive(buf, MSG_SIZE);
  if (did_receive) {
    // Serial.println("Received!");

    // Relay the data over LoRa radio
    send_buf();

    // Blink when sending
    if (telapsed > 200) {
      if (ison) {
        digitalWrite(LED_PIN, LOW);
      } else {
        digitalWrite(LED_PIN, HIGH);
      }
      ison = !ison;
      tblink_start = millis();
    }

    // Status bookkeeping
    // sending_printer.println("LoRa is sending data");
    if (is_stale) {
      // Serial.println("LoRa radio has started transmissions");
      is_stale = false;
    }
    timestamp_last_receive_ms = millis();
  } else {
    // Serial.println("Did not receive");

    // digitalWrite(LED_PIN, LOW);
    // Check if the process is stale (hasn't received data for a while)
    int time_since_last_receive_ms = millis() - timestamp_last_receive_ms;
    if (is_stale) {
      // stale_printer.println("LoRa process is stale.");

      // Turn LED off when stale
      digitalWrite(LED_PIN, LOW);
    } else {
      if (time_since_last_receive_ms > seconds_to_stale * 1000) {
        is_stale = true;
      }
    }
  }
}
