#ifndef IMU_helper_h
#define IMU_helper_h

#include <PacketSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "vicon_msg.pb.h"
#include "imu_msg.pb.h"


/*
 *  Encode IMU sensors into a protobuf
 */
void getImuInput(Adafruit_BNO055 &bno, messaging_IMU &input);

/*
 * Conver the double array of IMU data into a
 */
void displaySensorDetails(Adafruit_BNO055 & bno);

/*
 * Conver the double array of IMU data into a
 */
void displayCalStatus(Adafruit_BNO055 & bno);

/*
 * Conver the double array of IMU data into a
 */
void displaySensorReading(Adafruit_BNO055 & bno);

/*
 * Conver the double array of IMU data into a
 */
bool calibrateIMU(Adafruit_BNO055 & bno);



#endif
