#include "quad_utils.hpp"

// Pin setup
#define LED_PIN 13

// Options
constexpr bool kWaitForSerial = 1;
constexpr int kMaxBufferSize = 200;
const int kHeartbeatTimeoutMs = 200;

// Globals
uint8_t buf[kMaxBufferSize];
rexquad::Heartbeat g_heartbeat;

/////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial1.begin(256000);
  if (kWaitForSerial) {
    while (!Serial) {
      rexquad::Blink(LED_PIN, 100, 1);
    }
  }
  Serial.println("Connected to onboard Teensy!");

  // Setup Heartbeat
  g_heartbeat.SetTimeoutMs(kHeartbeatTimeoutMs);
  g_heartbeat.Activate();
}

/////////////////////////////////////////////
// Loop 
/////////////////////////////////////////////
void loop() {
  // Check for message over serial from onboard Feather
  int bytes_available = Serial1.available();
  if (bytes_available) {
    g_heartbeat.Pulse();
    int bytes_received = Serial1.readBytes((char*)buf, bytes_available);
    Serial.write(buf, bytes_received);
  }

  // Heartbeat indicator
  if (g_heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}