#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PacketSerial.h>

#include "imu_vicon_relay.hpp"
#include "pose.hpp"

#define LED_PIN     13

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

// Vicon
constexpr int MSG_SIZE = sizeof(rexlab::Pose<int32_t>);
uint8_t lora_buffer[MSG_SIZE];

// Build buffers and message types
constexpr int IMU_VICON_MSG_SIZE = sizeof(IMU_VICON);
uint8_t imu_vicon_buffer[IMU_VICON_MSG_SIZE];

// Message type
IMU_VICON imu_vicon = IMU_VICON_init_zero;

// Initialize packet serial ports
PacketSerial jetsonPacketSerial;
void sendJetsonMessage(PacketSerial &myPacketSerial, IMU_VICON &imu_vicon);

// Startup
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    //
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

void loop()
{
    jetsonPacketSerial.update();
    // Read in VICON measurement
    if (hasLoRaRecieved())
    {
        updateVicon(imu_vicon);
    }
    // Read in IMU measurement
    updateIMU(bno, imu_vicon);
    // Send IMU/Vicon Message
    sendJetsonMessage(jetsonPacketSerial, imu_vicon);
    //
    if (jetsonPacketSerial.overflow())
    {
        digitalWrite(LED_PIN, HIGH);
    }
    else
    {
        digitalWrite(LED_PIN, LOW);
    }
}

/*
 * Relay the message over from LoRa/IMU to Jetson
 */
void sendJetsonMessage(PacketSerial &myPacketSerial, IMU_VICON &imu_vicon)
{
    // displayImuVicon(imu_vicon);
    size_t msg_size = sizeof(IMU_VICON);
    memcpy(imu_vicon_buffer, &imu_vicon, msg_size);

    myPacketSerial.send(imu_vicon_buffer, msg_size);
    // uint8_t zero_byte = 0;
    // Serial.write(zero_byte);
}



