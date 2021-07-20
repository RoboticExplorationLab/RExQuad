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
	input.time = NULL;
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
bool loadSensorOffset(Adafruit_BNO055 & bno) {
	int eeAddress = 0;
	long bnoID;
	bool foundCalib = false;

	EEPROM.get(eeAddress, bnoID); // EEPROM address 0 stores the ID of the I2C device

	adafruit_bno055_offsets_t calibrationData;
	sensor_t sensor;

	bno.getSensor(&sensor);

	// Look for a previous calibration for bno055 in bnoID
	if (bnoID != sensor.sensor_id) {
		Serial.println("\nNo Calibration Data for this sensor exists in EEPROM.");
		foundCalib = false;
	}
	else {
		Serial.println("\nFound Calibration for this sensor A in EEPROM.");
		eeAddress += sizeof(long);
		EEPROM.get(eeAddress, calibrationData);

		Serial.println("Restoring Calibration data to the BNO055..");
		bno.setSensorOffsets(calibrationData);
		while (!bno.isFullyCalibrated()) {
			Serial.println("Setting BNO055 Calibration... Was the sensor properly calibrated?");
            displayCalStatus(bno);
			delay(1000);
		}

		Serial.println("Calibration data loaded into BNO055");
		foundCalib = true;
	}
	delay(1000);

	return foundCalib;
}

/*
 * Conver the double array of IMU data into a
 */
bool saveCalibration(Adafruit_BNO055 & bno) {
	adafruit_bno055_offsets_t newCali;
	bno.getSensorOffsets(newCali);
	Serial.print("\n\nStoring calibration data to EEPROM...");

	int eeAddress = 0;
	sensor_t sensor;
	long bnoID;
	bno.getSensor(&sensor);
	bnoID = sensor.sensor_id;

	// Store Sensor ID
	EEPROM.put(eeAddress, bnoID);
	// Store the calibration data type
	eeAddress += sizeof(long);
	EEPROM.put(eeAddress, newCali);

	delay(1000);
    
    Serial.println(" Saved!");

	return true;
}

/*
 * Conver the double array of IMU data into a
 */
bool calibrateIMU(Adafruit_BNO055 & bno, bool forceCalibration) {
	bool foundCalib = loadSensorOffset(bno);

	if (!forceCalibration && foundCalib && bno.isFullyCalibrated()) {
		return true;
	}
	else {
		bno.setExtCrystalUse(true);
		sensors_event_t event;

		Serial.println("Sensor needs to be calibrated: ");
		while(!bno.isFullyCalibrated()) {
			bno.getEvent(&event);
			displayCalStatus(bno);
			delay(100);
		}
        Serial.print("Calibration status: "); Serial.print(bno.isFullyCalibrated());

		saveCalibration(bno);
        return true;
	}
}
