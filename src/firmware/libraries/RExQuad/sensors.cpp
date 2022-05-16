#include "sensors.hpp"

#include <inttypes.h>

#include "messages.hpp"

namespace rexquad {
IMUSimulated::IMUSimulated() {
  accel_.version = sizeof(sensors_event_t);
  gyro_.version = sizeof(sensors_event_t);
  temp_.version = sizeof(sensors_event_t);
  int32_t sensor_id = 54321;
  accel_.sensor_id = sensor_id;
  gyro_.sensor_id = sensor_id;
  temp_.sensor_id = sensor_id;
  int32_t sensor_type = 54321;
  accel_.type = sensor_type;
  gyro_.type = sensor_type;
  temp_.type = sensor_type;
  temp_.temperature = 0.0;
}

void IMUSimulated::ReadSensor() {
  // TODO: make this a utility function
  const int msg_size = sizeof(MeasurementMsg) + 1;
  const uint8_t msg_id = MeasurementMsg::MsgID;
  int bytes_available = Serial.available();
  // if (bytes_available > msg_size) {
  if (1) {
    int bytes_received = Serial.readBytes((char*)recvbuf_, bytes_available);
    int start_index = 0;
    for (int i = 0; i < bytes_received; ++i) {
      if (recvbuf_[i] == msg_id) {
        start_index = i;
        break;
      }
    }
    // Copy memory into message buffer
    memcpy(msgbuf_, recvbuf_ + start_index, msg_size);

    // Copy to measurement measurement types
    MeasurementMsgFromBytes(msg_, recvbuf_);

    // Copy measurement message to sensor_event_t types
    accel_.acceleration.x = msg_.ax;
    accel_.acceleration.y = msg_.ay;
    accel_.acceleration.z = msg_.az;
    gyro_.gyro.x = msg_.wx;
    gyro_.gyro.y = msg_.wy;
    gyro_.gyro.z = msg_.wz;

    imumsg_.ax = msg_.ax;
    imumsg_.ay = msg_.ay;
    imumsg_.az = msg_.az;
    imumsg_.wx = msg_.wx;
    imumsg_.wy = msg_.wy;
    imumsg_.wz = msg_.wz;
  }
}

const sensors_event_t& IMUSimulated::GetAccel() const { return accel_; }

const sensors_event_t& IMUSimulated::GetGyro() const { return gyro_; }

const sensors_event_t& IMUSimulated::GetTemp() const { return temp_; }

const IMUMeasurementMsg& IMUSimulated::GetMeasurement() const {
  return imumsg_;
}

IMU::IMU(int pin_cs) : pin_cs_(pin_cs) {}

bool IMU::Connect() { return dso32_.begin_SPI(pin_cs_); }

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
  switch (rate) {
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
  imumsg_.ax = accel_.acceleration.x; 
  imumsg_.ay = accel_.acceleration.y; 
  imumsg_.az = accel_.acceleration.z; 
  imumsg_.wx = gyro_.gyro.x;
  imumsg_.wy = gyro_.gyro.y;
  imumsg_.wz = gyro_.gyro.z;
}

const sensors_event_t& IMU::GetAccel() const { return accel_; }

const sensors_event_t& IMU::GetGyro() const { return gyro_; }

const sensors_event_t& IMU::GetTemp() const { return temp_; }

const IMUMeasurementMsg& IMU::GetMeasurement() const {
  return imumsg_;
}

}  // namespace rexquad