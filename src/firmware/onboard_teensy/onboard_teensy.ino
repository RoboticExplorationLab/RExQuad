#include <RHHardwareSPI.h> 

#include "constants.hpp"
#include "messages.hpp"
#include "quad_utils.hpp"
#include "sensors.hpp"
#include "pose.hpp"
#include "serial_utils.hpp"
#define RF69_FREQ 910.0

// Pin setup
#define LED_PIN 13

// Radio Wing
#define RFM69_CS 10   // "F"
#define RFM69_INT 9  // "D"
#define RFM69_RST 8  // "C"
RH_RF69 rf69(RFM69_CS, RFM69_INT, hardware_spi);

// Options
constexpr bool kWaitForSerial = 0;
constexpr int kMaxBufferSize = 200;
const int kHeartbeatTimeoutMs = 200;

// Aliases
using Time = uint64_t;
using StateMsg = rexquad::StateMsg;
using ControlMsg = rexquad::ControlMsg;
using StateControl = rexquad::StateControlMsg;

// Constants
constexpr int kStateMsgSize = sizeof(StateMsg) + 1;
constexpr int kControlMsgSize = sizeof(ControlMsg) + 1;
constexpr int kStateControlSize = sizeof(StateControl) + 1;

// Globals
uint8_t g_bufrecv[kMaxBufferSize];  // buffer for receiving state estimate from feather
uint8_t g_statebuf[kStateMsgSize];
uint8_t g_bufrx[kMaxBufferSize];  // buffer for receiving radio messages from base station
uint8_t g_buftx[kStateControlSize];  // buffer for sending radio messages to base station

StateMsg g_statemsg;
ControlMsg g_controlmsg;
StateControl g_statecontrolmsg;  // for sending back to base station
rexquad::Heartbeat g_heartbeat;

// Controller
rexquad::StateVector xhat;  // state estimate
rexquad::InputVector u;
rexquad::ErrorVector e;  // error state

/////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
  pinMode(RFM69_RST, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  Serial1.begin(256000);
  if (kWaitForSerial) {
    while (!Serial) {
      rexquad::Blink(LED_PIN, 100, 1);
    }
  }
  Serial.println("Connected to onboard Teensy!");

  // Initialize Packet Radio
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    digitalWrite(RFM69_RST, HIGH);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(RFM69_RST, LOW);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  rf69.setFrequency(RF69_FREQ);
  rf69.setModemConfig(RH_RF69::GFSK_Rb250Fd250);
  Serial.println("RFM69 radio init successful!");
  // rexquad::InitRadio(rf69, RF69_FREQ, RFM69_RST, LED_PIN, /*encrypt=*/false);

  // Setup Heartbeat
  g_heartbeat.SetTimeoutMs(kHeartbeatTimeoutMs);
  g_heartbeat.Activate();
}

/////////////////////////////////////////////
// Loop 
/////////////////////////////////////////////
int packets_received = 0;
int states_received = 0;
void loop() {
  // Receive messages from base station
  bool pose_received = false;
  if (rf69.available()) {
    uint8_t len_rx = sizeof(g_bufrx);

    if (rf69.recv(g_bufrx, &len_rx)) {
      // Time t_mocap_us = curtime_us();
      ++packets_received;
      pose_received = true;

      Serial.print("Received message from base station! Number of bytes = ");
      Serial.println(len_rx);
    }
  }

  // Check for state estimate message over serial from onboard Feather
  int bytes_available = Serial1.available();
  bool state_received = false;
  if (bytes_available >= kStateMsgSize) {
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
      state_received = true;
      ++states_received;
      memcpy(g_statebuf, g_bufrecv+start_index, kStateMsgSize);
      rexquad::StateMsgFromBytes(g_statemsg, g_statebuf, 0);
      // Serial.print("Got state message!");
      // Serial.print("  position = [");
      // Serial.print(g_statemsg.x, 3);
      // Serial.print(", ");
      // Serial.print(g_statemsg.y, 3);
      // Serial.print(", ");
      // Serial.print(g_statemsg.z, 3);
      // Serial.println("]");
    }
  }

  // TODO: Calculate control

  // Send message to base station
  if (state_received) {
    const int rate_limiter = 2;
    if (states_received % (2*rate_limiter) == 0) {
      rexquad::RatePrinter();
      rexquad::ControlMsgFromVector(g_controlmsg, u.data());
      rexquad::ControlMsgToBytes(g_controlmsg, g_buftx);
      rf69.send(g_buftx, kControlMsgSize);
    } else if (states_received % (2*rate_limiter) == rate_limiter) {
      rexquad::StateMsgFromVector(g_statemsg, xhat.data());
      rexquad::StateMsgToBytes(g_statemsg, g_buftx);
      rf69.send(g_buftx, kStateMsgSize);
    }
    // rf69.waitPacketSent();  // needed?
    // Serial.println("Sent message to base station.");
  }
  // if (state_received) {
  //   String message = "Hello from Teensy";
  //   rf69.send(message.c_str(), message.length());
  //   Serial.println("Sent message over radio");
  // }

  // Heartbeat indicator
  if (g_heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}