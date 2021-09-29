#include "IMU_helper.h"

/*
 *  Encode IMU sensors into a protobuf IMU message
 */
void getImuInput(Adafruit_BNO055 &bno, messaging_IMU &input) {
	imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	// Populate translational acceleration
	input.acc_x = acc.x();
	input.acc_y = acc.y();
	input.acc_z = acc.z();
	// Populate rotational velocity
	input.gyr_x = gyr.x();
	input.gyr_y = gyr.y();
	input.gyr_z = gyr.z();
	// Make sure the publisher fills in the proper time
	input.time = 0.0;
}

/*
 * Conver the double array of IMU data into a
 */
void displayCalStatus(Adafruit_BNO055 & bno) {
	uint8_t system, gyro, accel, mag = 0;
	bno.getCalibration(&system, &gyro, &accel, &mag);
	/* Display the individual values */
	Serial.println("\n-----------Sensor Calibration-----------");
	Serial.print("    Sys:"); Serial.print(system, DEC);
	Serial.print("    Gyr:"); Serial.print(gyro, DEC);
	Serial.print("    Acc:"); Serial.print(accel, DEC);
	Serial.print("    Mag:"); Serial.print(mag, DEC);
	Serial.println("\n----------------------------------------");
}

/*
 * Conver the double array of IMU data into a
 */
void displaySensorReading(Adafruit_BNO055 & bno) {
	imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

	/* Display the individual values */
	Serial.println("\n-------------Sensor Reading-------------");
	Serial.print(" Gyr:");
	Serial.print(gyr.x(), DEC); Serial.print(" ");
	Serial.print(gyr.y(), DEC); Serial.print(" ");
	Serial.print(gyr.z(), DEC); Serial.print("\n");
	Serial.print(" Acc:");
	Serial.print(acc.x(), DEC); Serial.print(" ");
	Serial.print(acc.y(), DEC); Serial.print(" ");
	Serial.print(acc.z(), DEC); Serial.print("\n");
	Serial.print(" Mag:");
	Serial.print(mag.x(), DEC); Serial.print(" ");
	Serial.print(mag.y(), DEC); Serial.print(" ");
	Serial.print(mag.z(), DEC); Serial.print("  ");
	Serial.println("\n----------------------------------------");
}

/*
 * Conver the double array of IMU data into a
 */
bool calibrateIMU(Adafruit_BNO055 & bno) {
	bno.setExtCrystalUse(true);
	sensors_event_t event;

	adafruit_bno055_offsets_t calibrationData = {15293, 0, 615, 50, 0, 0, 8192, 512, 0, 0, 512};
	bno.setSensorOffsets(calibrationData);
	Serial.println("Calibration data loaded into BNO055");
	delay(1000);

	Serial.println("Checking Sensor Calibration: ");
	while(!bno.isFullyCalibrated()) {
		bno.getEvent(&event);
		displayCalStatus(bno);
		delay(100);
	}
	Serial.print("Calibration status: "); Serial.print(bno.isFullyCalibrated());

	return true;
}
