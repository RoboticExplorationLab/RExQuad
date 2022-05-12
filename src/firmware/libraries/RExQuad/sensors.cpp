#include "sensors.hpp"

namespace rexquad {

IMU::IMU(int pin_cs) : pin_cs_(pin_cs) {}

bool IMU::Connect() {
  return dso32_.begin_SPI(pin_cs_);
}

void IMU::SetAccelRange(AccelRange range) {
  switch (range) {
    case AccelRange::Accel4g:
      dso32_.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
      break;
    case AccelRange::Accel8g:
      dso32_.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
      break;
    case AccelRange::Accel16g:
      dso32_.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
      break;
    case AccelRange::Accel32g:
      dso32_.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
      break;
  }
}

void IMU::SetAccelRate(DataRate rate) {
  switch(rate) {
    case DataRate::Rate0Hz:
      dso32_.setAccelDataRate(LSM6DS_RATE_SHUTDOWN);
      break;
    case DataRate::Rate12_5Hz:
      dso32_.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
      break;
    case DataRate::Rate26Hz:
      dso32_.setAccelDataRate(LSM6DS_RATE_26_HZ);
      break;
    case DataRate::Rate52Hz:
      dso32_.setAccelDataRate(LSM6DS_RATE_52_HZ);
      break;
    case DataRate::Rate104Hz:
      dso32_.setAccelDataRate(LSM6DS_RATE_104_HZ);
      break;
    case DataRate::Rate208Hz:
      dso32_.setAccelDataRate(LSM6DS_RATE_208_HZ);
      break;
    case DataRate::Rate416Hz:
      dso32_.setAccelDataRate(LSM6DS_RATE_416_HZ);
      break;
    case DataRate::Rate833Hz:
      dso32_.setAccelDataRate(LSM6DS_RATE_833_HZ);
      break;
    case DataRate::Rate1_66KHz:
      dso32_.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
      break;
    case DataRate::Rate3_33KHz:
      dso32_.setAccelDataRate(LSM6DS_RATE_3_33K_HZ);
      break;
    case DataRate::Rate6_66KHz:
      dso32_.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
      break;
  }
}

void IMU::SetGyroRange(GyroRange range) {
  switch (range) {
    case GyroRange::Gyro125dps:
      dso32_.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
      break;
    case GyroRange::Gyro250dps:
      dso32_.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
      break;
    case GyroRange::Gyro500dps:
      dso32_.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
      break;
    case GyroRange::Gyro1000dps:
      dso32_.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
      break;
    case GyroRange::Gyro2000dps:
      dso32_.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
      break;
  }
}

void IMU::SetGyroRate(DataRate rate) {
  switch (rate) {
    case DataRate::Rate0Hz:
      dso32_.setGyroDataRate(LSM6DS_RATE_SHUTDOWN);
      break;
    case DataRate::Rate12_5Hz:
      dso32_.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
      break;
    case DataRate::Rate26Hz:
      dso32_.setGyroDataRate(LSM6DS_RATE_26_HZ);
      break;
    case DataRate::Rate52Hz:
      dso32_.setGyroDataRate(LSM6DS_RATE_52_HZ);
      break;
    case DataRate::Rate104Hz:
      dso32_.setGyroDataRate(LSM6DS_RATE_104_HZ);
      break;
    case DataRate::Rate208Hz:
      dso32_.setGyroDataRate(LSM6DS_RATE_208_HZ);
      break;
    case DataRate::Rate416Hz:
      dso32_.setGyroDataRate(LSM6DS_RATE_416_HZ);
      break;
    case DataRate::Rate833Hz:
      dso32_.setGyroDataRate(LSM6DS_RATE_833_HZ);
      break;
    case DataRate::Rate1_66KHz:
      dso32_.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
      break;
    case DataRate::Rate3_33KHz:
      dso32_.setGyroDataRate(LSM6DS_RATE_3_33K_HZ);
      break;
    case DataRate::Rate6_66KHz:
      dso32_.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
      break;
  }
}

void IMU::ReadSensor() {
  dso32_.getEvent(&accel_, &gyro_, &temp_);
}

sensors_event_t IMU::GetAccel() {
  return accel_;
}

sensors_event_t IMU::GetGyro() {
  return gyro_;
}

sensors_event_t IMU::GetTemp() {
  return temp_;
}

}  // namespace rexquad