#pragma once

#include <Adafruit_LSM6DSO32.h>

namespace rexquad {

class IMUBase {

public:
  virtual void ReadSensor() = 0;
  virtual const sensors_event_t& GetAccel() const = 0;
  virtual const sensors_event_t& GetGyro() const = 0;
  virtual const sensors_event_t& GetTemp() const = 0;

  template <class Serial>
  void PrintData(Serial serial) {
    sensors_event_t accel = GetAccel();
    sensors_event_t temp = GetTemp();
    sensors_event_t gyro = GetGyro();

    serial.print("\t\tTemperature ");
    serial.print(temp.temperature);
    serial.println(" deg C");

    /* Display the results (acceleration is measured in m/s^2) */
    serial.print("\t\tAccel X: ");
    serial.print(accel.acceleration.x);
    serial.print(" \tY: ");
    serial.print(accel.acceleration.y);
    serial.print(" \tZ: ");
    serial.print(accel.acceleration.z);
    serial.println(" m/s^2 ");

    /* Display the results (rotation is measured in rad/s) */
    serial.print("\t\tGyro X: ");
    serial.print(gyro.gyro.x);
    serial.print(" \tY: ");
    serial.print(gyro.gyro.y);
    serial.print(" \tZ: ");
    serial.print(gyro.gyro.z);
    serial.println(" radians/s ");
    serial.println();
  }

};

class IMU : public IMUBase {
 public:
  enum class AccelRange { Accel4g, Accel8g, Accel16g, Accel32g };
  enum class GyroRange { Gyro125dps, Gyro250dps, Gyro500dps, Gyro1000dps, Gyro2000dps };
  enum class DataRate {
    Rate0Hz,
    Rate12_5Hz,
    Rate26Hz,
    Rate52Hz,
    Rate104Hz,
    Rate208Hz,
    Rate416Hz,
    Rate833Hz,
    Rate1_66KHz,
    Rate3_33KHz,
    Rate6_66KHz
  };

  IMU(int pin_cs);
  bool Connect();

  void SetAccelRange(AccelRange range);
  void SetAccelRate(DataRate rate);
  void SetGyroRange(GyroRange range);
  void SetGyroRate(DataRate rate);

  void ReadSensor();
  const sensors_event_t& GetAccel() const override;
  const sensors_event_t& GetTemp() const override;
  const sensors_event_t& GetGyro() const override;

  template <class Serial>
  void PrintSettings(Serial serial) {
    serial.print("Accelerometer range set to: ");
    switch (dso32_.getAccelRange()) {
      case LSM6DSO32_ACCEL_RANGE_4_G:
        serial.println("+-4G");
        break;
      case LSM6DSO32_ACCEL_RANGE_8_G:
        serial.println("+-8G");
        break;
      case LSM6DSO32_ACCEL_RANGE_16_G:
        serial.println("+-16G");
        break;
      case LSM6DSO32_ACCEL_RANGE_32_G:
        serial.println("+-32G");
        break;
    }

    serial.print("Gyro range set to: ");
    switch (dso32_.getGyroRange()) {
      case LSM6DS_GYRO_RANGE_125_DPS:
        serial.println("125 degrees/s");
        break;
      case LSM6DS_GYRO_RANGE_250_DPS:
        serial.println("250 degrees/s");
        break;
      case LSM6DS_GYRO_RANGE_500_DPS:
        serial.println("500 degrees/s");
        break;
      case LSM6DS_GYRO_RANGE_1000_DPS:
        serial.println("1000 degrees/s");
        break;
      case LSM6DS_GYRO_RANGE_2000_DPS:
        serial.println("2000 degrees/s");
        break;
      case ISM330DHCX_GYRO_RANGE_4000_DPS:
        break;  // unsupported range for the DSO32
    }

    serial.print("Accelerometer data rate set to: ");
    switch (dso32_.getAccelDataRate()) {
      case LSM6DS_RATE_SHUTDOWN:
        serial.println("0 Hz");
        break;
      case LSM6DS_RATE_12_5_HZ:
        serial.println("12.5 Hz");
        break;
      case LSM6DS_RATE_26_HZ:
        serial.println("26 Hz");
        break;
      case LSM6DS_RATE_52_HZ:
        serial.println("52 Hz");
        break;
      case LSM6DS_RATE_104_HZ:
        serial.println("104 Hz");
        break;
      case LSM6DS_RATE_208_HZ:
        serial.println("208 Hz");
        break;
      case LSM6DS_RATE_416_HZ:
        serial.println("416 Hz");
        break;
      case LSM6DS_RATE_833_HZ:
        serial.println("833 Hz");
        break;
      case LSM6DS_RATE_1_66K_HZ:
        serial.println("1.66 KHz");
        break;
      case LSM6DS_RATE_3_33K_HZ:
        serial.println("3.33 KHz");
        break;
      case LSM6DS_RATE_6_66K_HZ:
        serial.println("6.66 KHz");
        break;
    }

    serial.print("Gyro data rate set to: ");
    switch (dso32_.getGyroDataRate()) {
      case LSM6DS_RATE_SHUTDOWN:
        serial.println("0 Hz");
        break;
      case LSM6DS_RATE_12_5_HZ:
        serial.println("12.5 Hz");
        break;
      case LSM6DS_RATE_26_HZ:
        serial.println("26 Hz");
        break;
      case LSM6DS_RATE_52_HZ:
        serial.println("52 Hz");
        break;
      case LSM6DS_RATE_104_HZ:
        serial.println("104 Hz");
        break;
      case LSM6DS_RATE_208_HZ:
        serial.println("208 Hz");
        break;
      case LSM6DS_RATE_416_HZ:
        serial.println("416 Hz");
        break;
      case LSM6DS_RATE_833_HZ:
        serial.println("833 Hz");
        break;
      case LSM6DS_RATE_1_66K_HZ:
        serial.println("1.66 KHz");
        break;
      case LSM6DS_RATE_3_33K_HZ:
        serial.println("3.33 KHz");
        break;
      case LSM6DS_RATE_6_66K_HZ:
        serial.println("6.66 KHz");
        break;
    }
  }

 private:
  int pin_cs_;
  sensors_event_t accel_;
  sensors_event_t gyro_;
  sensors_event_t temp_;
  Adafruit_LSM6DSO32 dso32_;
};

}  // namespace rexquad