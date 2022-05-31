#include <RHHardwareSPI.h> 

#include "constants.hpp"
#include "messages.hpp"
#include "quad_utils.hpp"
#include "sensors.hpp"
#include "pose.hpp"
#define RF69_FREQ 915.0

// Pin setup
#define LED_PIN 13

// Radio Wing
#define RFM69_CS 10   // "F"
#define RFM69_INT 9  // "D"
#define RFM69_RST 8  // "C"
RH_RF69 rf69(RFM69_CS, RFM69_INT, hardware_spi);

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
uint8_t g_bufrx[kMaxBufferSize];

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
  // SPI1.begin();
  // Serial.println("SPI started.");
  // digitalWrite(RFM69_RST, HIGH);
  // delay(10);
  // digitalWrite(RFM69_RST, LOW);
  // delay(10);
  // if (!rf69.init()) {
  //   Serial.println("RFM69 radio init failed");
  //   digitalWrite(RFM69_RST, HIGH);
  //   digitalWrite(LED_PIN, HIGH);
  //   delay(100);
  //   digitalWrite(RFM69_RST, LOW);
  //   digitalWrite(LED_PIN, LOW);
  //   delay(100);
  // }
  // rf69.setFrequency(RF69_FREQ);
  // rf69.setModemConfig(RH_RF69::GFSK_Rb250Fd250);
  // Serial.println("RFM69 radio init successful!");
  rexquad::InitRadio(rf69, RF69_FREQ, RFM69_RST, LED_PIN, /*encrypt=*/false);

  // Setup Heartbeat
  g_heartbeat.SetTimeoutMs(kHeartbeatTimeoutMs);
  g_heartbeat.Activate();
}

/////////////////////////////////////////////
// Loop 
/////////////////////////////////////////////
int packets_received = 0;
void loop() {
  // Process MOCAP pose
  bool pose_received = false;
  if (rf69.available()) {
    uint8_t len_mocap = sizeof(g_bufrx);
    // Serial.println("Radio packet available!");

    if (rf69.recv(g_bufrx, &len_mocap)) {
      // Time t_mocap_us = curtime_us();
      ++packets_received;
      pose_received = true;
      g_heartbeat.Pulse();

      // Convert bytes into pose message
      // rexquad::PoseFromBytes(pose_mocap, (char*)buf_mocap);

      Serial.print("Received Pose Message over radio! Number of bytes = ");
      Serial.println(len_mocap);
      // // Update State Estimate
      // filter.PoseMeasurement(pose_mocap, t_mocap_us);
    }
  }

  // Check for message over serial from onboard Feather
  int bytes_available = Serial1.available();
  if (bytes_available >= kStateMsgSize) {
    g_heartbeat.Pulse();
    int bytes_received = Serial1.readBytes((char*)g_bufrecv, bytes_available);
    // Serial.write(g_bufrecv, bytes_received)
    int start_index = 0;
    int msgid = StateMsg::MsgID;  
    for (int i = 0; i < bytes_received; ++i) {
      if (g_bufrecv[i] == msgid) {
        start_index = i;
        break;
      }
    }
    // Serial.print("Number of bytes received: ");
    // Serial.println(bytes_received);
    
    // Copy received message to state estimate
    if (g_bufrecv[start_index] == msgid) {
      memcpy(g_statebuf, g_bufrecv+start_index, kStateMsgSize);
      rexquad::StateMsgFromBytes(g_statemsg, g_statebuf, 0);
      Serial.print("Got state message!");
      // Serial.print(start_index);
      // Serial.print("  payload = [ ");
      // for (int i = 0; i < 3 * sizeof(float) + 1; ++i) {
      //   Serial.print(g_bufrecv[i], HEX);
      //   Serial.print(" ");
      // }
      // Serial.println("]");
      Serial.print("  position = [");
      Serial.print(g_statemsg.x, 3);
      Serial.print(", ");
      Serial.print(g_statemsg.y, 3);
      Serial.print(", ");
      Serial.print(g_statemsg.z, 3);
      Serial.println("]");
    }
  }

  // Heartbeat indicator
  if (g_heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}