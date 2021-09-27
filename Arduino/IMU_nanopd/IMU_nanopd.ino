#include "IMU_helper.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PacketSerial.h>
// #include <Pose.hpp>

#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include "pose.hpp"
#include "receiver.hpp"

// Global Constants
#define RFM95_CS    8
#define RFM95_RST   4
#define RFM95_INT   3
#define RF95_FREQ   915.0

#define LED_PIN     13
// #define MSG_SIZE    24

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
sensors_event_t imu_event;

// Vicon
constexpr int MSG_SIZE = sizeof(rexlab::Pose<int32_t>);
uint8_t lora_buffer[MSG_SIZE];

// Build buffers and message types
uint8_t imu_buffer[256];
size_t imu_buffer_length = sizeof(imu_buffer);

uint8_t imu_vicon_buffer[512];
size_t imu_vicon_buffer_length = sizeof(imu_vicon_buffer);

// Protobuf types
messaging_IMU IMU_input = messaging_IMU_init_zero;
messaging_VICON VICON_measurement = messaging_VICON_init_zero;
messaging_IMU_VICON IMU_VICON_message = {true, IMU_input, true, VICON_measurement};

// Initialize packet serial ports
PacketSerial jetsonPacketSerial;

void sendMessage(PacketSerial &myPacketSerial, messaging_IMU_VICON &mes);

// Startup
void setup() {
    pinMode(LED_PIN, OUTPUT);

    if (!initialize_LoRaViconReceiver(lora_buffer, MSG_SIZE))
    {
        Serial.println("Failed to properly setup LoRaViconReceiver!!");
        while(true)
        {
        }
    }

    // Start up SPI IMU and initialize
    if (!bno.begin())
    {
        Serial.print("\nOoops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
        while (true)
        {
        }
    }

    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }
    jetsonPacketSerial.setStream(&Serial);

    randomSeed(42);
}

void loop() {
    jetsonPacketSerial.update();

    if (hasLoRaRecieved())
    {
        updateViconProto(VICON_measurement);
    }

    // Read in IMU measurement
    bno.getEvent(&imu_event);
    getImuInput(bno, IMU_input);

    // Send IMU/Vicon Message
    IMU_VICON_message = {true, IMU_input, true, VICON_measurement};
    sendJetsonMessage(IMU_VICON_message);

    if (jetsonPacketSerial.overflow())
    {
        digitalWrite(LED_PIN, HIGH);
    }
    else {
        digitalWrite(LED_PIN, LOW);
    }
}

/*
 * Relay the message over from Holybro to Jetson
 */
void sendJetsonMessage(messaging_IMU_VICON & mes) {
    /* Create a stream that will write to our buffer. */
    pb_ostream_t stream = pb_ostream_from_buffer(imu_vicon_buffer, imu_vicon_buffer_length);

    int status = pb_encode(&stream, messaging_IMU_VICON_fields, &mes);
    if (status)
    {
        int message_length = stream.bytes_written;
        jetsonPacketSerial.send(imu_vicon_buffer, message_length);
    }
}

