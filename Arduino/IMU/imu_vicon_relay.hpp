#ifndef imu_vicon_relay_hpp
#define imu_vicon_relay_hpp

#include <cstdint>
#include <functional>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define RF95_FREQ 915.0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

typedef struct _IMU_VICON
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;

    float pos_x;
    float pos_y;
    float pos_z;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;

    uint32_t time;
} IMU_VICON;

#define IMU_VICON_init_zero                      \
    {                                            \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 \
    }

bool initialize_LoRaViconReceiver(uint8_t *buf, size_t msg_size);

void onLoRaReceive(int packetSize);

bool hasLoRaRecieved();

void updateVicon(IMU_VICON &imu_vicon);

void updateIMU(Adafruit_BNO055 &bno, IMU_VICON &imu_vicon);

void displaySensorDetails(Adafruit_BNO055 &bno);

void displayCalStatus(Adafruit_BNO055 &bno);

void displaySensorReading(Adafruit_BNO055 &bno);

bool calibrateIMU(Adafruit_BNO055 &bno);

void displayImuVicon(IMU_VICON &imu_vicon);

#endif
