#include "constants.hpp"
#include "messages.hpp"
#include "quad_utils.hpp"

// Pin setup
#define LED_PIN 13

// Options
constexpr bool kWaitForSerial = 1;
constexpr int kMaxBufferSize = 200;
const int kHeartbeatTimeoutMs = 1500;

// Aliases
using Time = uint64_t;
using StateMsg = rexquad::StateMsg;
using StateControl = rexquad::StateControlMsg;

// Constants
constexpr int kStateMsgSize = sizeof(StateMsg) + 1;
constexpr int kStateControlSize = sizeof(StateControl) + 1;

// Globals
uint8_t g_bufrecv[kMaxBufferSize];
uint8_t g_statebuf[kStateMsgSize];

StateMsg g_statemsg;
rexquad::Heartbeat g_heartbeat;

// Controller
rexquad::StateVector xhat;  // state estimate
rexquad::InputVector u;
rexquad::ErrorVector e;  // error state

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
    int bytes_received = Serial1.readBytes((char*)g_bufrecv, bytes_available);
    int start_index = 0;
    int msgid = StateMsg::MsgID;  
    for (int i = 0; i < bytes_received; ++i) {
      if (g_bufrecv[i] == msgid) {
        start_index = i;
        break;
      }
    }
    
    // Copy received message to state estimate
    if (g_bufrecv[start_index] == msgid) {
      memcpy(g_statebuf, g_bufrecv+start_index, kStateMsgSize);
      rexquad::StateMsgFromBytes(g_statemsg, g_statebuf, 0);
      Serial.println("Got state message!");
    }
  }

  // Heartbeat indicator
  if (g_heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}