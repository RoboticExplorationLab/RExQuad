#include "sensors.hpp"
#include "pose.hpp"
#include "quad_utils.hpp"

// Pin Setup
#define LED_PIN 13

// Options
const int kHeartbeatTimeoutMs = 1000;

// Constants
constexpr int kMaxBufferSize = 200;

// Globals
uint8_t buf[kMaxBufferSize];
rexquad::Heartbeat heartbeat;

/////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
  Serial.begin(256000);
  while (!Serial) {
    rexquad::Blink(LED_PIN, 100, 1);
  }
  delay(500);
  Serial.println("Welcome to the Heartbeat Test!");
  Serial.println("-------------------------------");
  Serial.println("Instructions:");
  Serial.println("  1. Send any command over serial to the Feather");
  Serial.println("  2. The LED should turn off if you don't send a command for 1 second.");
  Serial.println("  3. The LED should turn on once you send another command.");

  // Setup Heartbeat
  heartbeat.SetTimeoutMs(kHeartbeatTimeoutMs);
  heartbeat.Activate();

  digitalWrite(LED_PIN, LOW);
}

/////////////////////////////////////////////
// Loop 
/////////////////////////////////////////////
int packetnum = 0;
void loop() {
  // Check for data over Serial
  int bytes_available = Serial.available();
  if (bytes_available > 0) { 
    Serial.readBytes((char*)buf, bytes_available);
    heartbeat.Pulse();
    // TODO: Turn LED on/off if it's stale
    // Blink(LED_PIN, 20, 1);
  }
  if (heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
  return;

}