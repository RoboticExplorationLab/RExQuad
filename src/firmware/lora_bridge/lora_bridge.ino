#include "constants.hpp"
#include "messages.hpp"
#include "pose.hpp"
#include "quad_utils.hpp"
#include "sensors.hpp"
#include "serial_utils.hpp"

// Pin Setup
#define LED_PIN 13

// M0 LoRa pinouts
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 910.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Options
enum RXOUTPUT {
  RATE,
  PRINTPOSE,
  SERIALPOSE
};
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
uint8_t buf_recv[kMaxBufferSize];
rexquad::Heartbeat heartbeat;

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

  // Init Radio
  rexquad::InitLoRa(rf95, RF95_FREQ, RFM95_RST, LED_PIN);

  // Setup Heartbeat
  heartbeat.SetTimeoutMs(kHeartbeatTimeoutMs);
  heartbeat.Activate();

  digitalWrite(LED_PIN, LOW);
}

/////////////////////////////////////////////
// Loop
/////////////////////////////////////////////
int packets_received = 0;
void loop() {
  int bytes_available = rf95.available();
  if (bytes_available) {
    Serial.print(bytes_available);
    Serial.println(" bytes available.");
    uint8_t len = sizeof(buf_recv);

    if (rf95.recv(buf_recv, &len)) {
      Serial.println("Packet Received!");
      heartbeat.Pulse();
    }
  }
  if (heartbeat.IsDead()) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
  // delay(100);
}