#include <sys/types.h>

#include "constants.hpp"
#include "messages.hpp"
#include "pose.hpp"
#include "quad_utils.hpp"
#include "sensors.hpp"
#include "serial_utils.hpp"

// Pin Setup
#define LED_PIN 13

// IMU (I2C)
#define LSM_SCL 21
#define LSM_SDA 20
rexquad::IMU imureal;

// Motors
// #define FRONT_LEFT_PIN 9
// #define FRONT_RIGHT_PIN 10  // CONFLICT!
// #define BACK_RIGHT_PIN 11 // CONFLICT!
// #define BACK_LEFT_PIN 12

// Radio Setup
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4
#define RF69_FREQ 915.0
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// LoRa Wing pinouts
#define RFM95_CS 10   // "B"
#define RFM95_RST 11  // "A"
// #define RFM95_INT 6
#define RFM95_INT 9
#define RF95_FREQ 914.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Options
enum RXOUTPUT { RATE, PRINTPOSE, SERIALPOSE };
constexpr int kWaitForSerial = 1;
constexpr int kConnectoToIMU = 0;
const int kHeartbeatTimeoutMs = 1000;
const RXOUTPUT output = RATE;

// Constants
using Pose = rexquad::PoseMsg;
using StateControl = rexquad::StateControlMsg;
constexpr int kMaxBufferSize = 200;
constexpr int kPoseSize = sizeof(Pose) + 1;
constexpr int kStateControlSize = sizeof(StateControl) + 1;

// Globals
uint8_t buf_mocap[kMaxBufferSize];
uint8_t buf_send[kStateControlSize];
Pose pose_mocap;
StateControl statecontrol_msg;
rexquad::Heartbeat heartbeat;

rexquad::StateVector xhat;
rexquad::InputVector u;

/////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
  Serial.begin(256000);
  if (kWaitForSerial) {
    while (!Serial) {
      rexquad::Blink(LED_PIN, 100, 1);
    }
    Serial.println("Connected to Receiver!");
  }

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  delay(100);

  Serial.println("Feather LoRa TX Test!");
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    digitalWrite(RFM69_RST, HIGH);
    delay(100);
    digitalWrite(RFM69_RST, LOW);
    delay(100);
  }
  rf69.setFrequency(RF69_FREQ);
  rf69.setModemConfig(RH_RF69::GFSK_Rb250Fd250);
  Serial.println("RFM69 radio init successful!");

  // manual reset

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    digitalWrite(RFM95_RST, LOW);
    delay(100);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // Setup Heartbeat
  heartbeat.SetTimeoutMs(kHeartbeatTimeoutMs);
  heartbeat.Activate();

  digitalWrite(LED_PIN, LOW);
  Serial.println("Starting loop...");
}

/////////////////////////////////////////////
// Loop
/////////////////////////////////////////////
int packets_received = 0;
char buf[13] = "Hello, LoRa!";
void loop() {
  // Serial.println("Sending packet...");
  // char buf[7] = "Hello!";
  // int len = sizeof(buf);
  // rf95.send((uint8_t*)buf, len);
  // // rf95.waitPacketSent();
  // Serial.println("Packet sent!");
  // rexquad::RatePrinter();

  if (rf69.available()) {
    uint8_t len_mocap = sizeof(buf_mocap);

    if (rf69.recv(buf_mocap, &len_mocap)) {
      ++packets_received;
      heartbeat.Pulse();

      // Convert bytes into pose message
      rexquad::PoseFromBytes(pose_mocap, (char*)buf_mocap);

      // Create StateControl Message
      statecontrol_msg.x = pose_mocap.x;
      statecontrol_msg.y = pose_mocap.y;
      statecontrol_msg.z = pose_mocap.z;
      statecontrol_msg.qw = pose_mocap.qw;
      statecontrol_msg.qx = pose_mocap.qx;
      statecontrol_msg.qy = pose_mocap.qy;
      statecontrol_msg.qz = pose_mocap.qz;
      for (int i = 0; i < rexquad::kNumInputs; ++i) {
        u(i) = packets_received;
        statecontrol_msg.u[i] = u(i);
      }

      // Send packet over LoRa
      if (packets_received % 10 == 0) {
        rexquad::StateControlMsgToBytes(statecontrol_msg, buf_send);
        rf95.send((uint8_t*) buf, sizeof(buf));
        // rf95.waitPacketSent();
      }

      switch (output) {
        case RATE:
          rexquad::RatePrinter();
          break;
        case PRINTPOSE:
          Serial.print("position = [");
          Serial.print(pose_mocap.x, 3);
          Serial.print(", ");
          Serial.print(pose_mocap.y, 3);
          Serial.print(", ");
          Serial.print(pose_mocap.z, 3);
          Serial.println("]");
          break;
        case SERIALPOSE:
          Serial.write(buf_send, kStateControlSize);
          break;
      }
    }
  }
  if (heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}