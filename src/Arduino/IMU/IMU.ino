#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "imu_vicon_relay.hpp"
#include "pose.hpp"
#include "crc8.h"

#define LED_PIN     13

// IMU
 Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

// Vicon
constexpr int MSG_SIZE = sizeof(rexlab::Pose<int32_t>);
uint8_t lora_buffer[MSG_SIZE];

// Build buffers and message types
constexpr int IMU_VICON_MSG_SIZE = sizeof(IMU_VICON) + 1;
uint8_t imu_vicon_buffer[IMU_VICON_MSG_SIZE];

// Message type
IMU_VICON imu_vicon = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0};

// Initialize packet serial ports
void sendJetsonMessage(IMU_VICON &imu_vicon);
CRC8_PARAMS crc8_params = DEFAULT_CRC8_PARAMS;

// Startup
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }

    // Start up the lora radio
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
}

void loop()
{
    // Limit to 1000 Hz
    // delay(100);

    // Read in VICON measurement
    if (hasLoRaReceived())
    {
        updateVicon(imu_vicon);
    }

    // Read in IMU measurement
    updateIMU(bno, imu_vicon);

    // Send IMU/Vicon Message
    // Serial.printf(" Quat: [%1.3f, %1.3f, %1.3f, %1.3f]\n", imu_vicon.quat_w, imu_vicon.quat_x, imu_vicon.quat_y, imu_vicon.quat_z);
    sendJetsonMessage(imu_vicon);
    // displayImuVicon(imu_vicon);
    // constraintCheck(imu_vicon);
}

/*
 * Relay the message over from LoRa/IMU to Jetson
 */
void sendJetsonMessage(IMU_VICON &imu_vicon)
{
    // Copy IMU_VICON message into buffer of size sizeof(IMU_VICON)+1
    size_t imu_vicon_size = sizeof(IMU_VICON);
    memcpy(imu_vicon_buffer, &imu_vicon, imu_vicon_size);
    // Compute the CRC8 value of the IMU_VICON message
    uint8_t crc = crc8(crc8_params, imu_vicon_buffer, imu_vicon_size);
    imu_vicon_buffer[imu_vicon_size] = crc;
    // Write IMU_VICON value along with crc8 value
    Serial.write(imu_vicon_buffer, IMU_VICON_MSG_SIZE);
}
