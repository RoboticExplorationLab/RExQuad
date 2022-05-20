#include <RH_RF95.h>
#include <SPI.h>

#include "quad_utils.hpp"
#include "serial_utils.hpp"

// LoRa pins
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 914.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define LED_PIN 13

// Constants
constexpr int kWaitForSerial = 1;
const int kHeartbeatTimeoutMs = 1000;

// Globals
rexquad::Heartbeat heartbeat;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(256000);
  if (kWaitForSerial) {
    while (!Serial) {
      rexquad::Blink(LED_PIN, 100, 1);
    }
    Serial.println("Connected to Receiver!");
  }
  delay(100);

  Serial.println("Feather LoRa RX Test!");

  // Initialize LoRa radio
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    rexquad::Blink(LED_PIN, 100, 10);
    digitalWrite(RFM95_RST, LOW);
    delay(100);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
  }
  rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    rexquad::Blink(LED_PIN, 100, 10);
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);

  // Setup Heartbeat
  heartbeat.SetTimeoutMs(kHeartbeatTimeoutMs);
  heartbeat.Activate();
}

void loop() {
  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      Serial.print("Got: ");
      Serial.println((char*)buf);
      rexquad::RatePrinter();
      heartbeat.Pulse();
    } else {
      Serial.println("Receive failed");
    }
  }
  if (heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}
