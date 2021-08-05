#include "IMU_helper.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PacketSerial.h>

#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>

#define HOLYBRO_BAUDRATE (57600)
#define LED_PIN (13)

// #define DEBUG (true)
#define DEBUG (false)

// #define FORCE_CALI (true)
#define FORCE_CALI (false)

#define LOOP_RATE_DELAY (1)

#define LED_PIN (13)

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
sensors_event_t imu_event;

// Build buffers and message types
size_t vicon_buffer_length = 256
uint8_t vicon_buffer[vicon_buffer_length];
size_t imu_buffer_length = 256
uint8_t imu_buffer[imu_buffer_length];

messaging_VICON VICON_measurement = messaging_VICON_init_zero;
messaging_IMU IMU_input = messaging_IMU_init_zero;
messaging_IMU_VICON IMU_VICON_message = {false, IMU_input, false, VICON_measurement};

// Initialize packet serial ports
PacketSerial jetsonPacketSerial;
PacketSerial viconPacketSerial;


//
void onViconRecieved(const uint8_t *buffer, size_t size);

//
void sendMessage(PacketSerial &myPacketSerial, messaging_IMU_VICON &mes);


void setup() {
    pinMode(LED_PIN, OUTPUT);

    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(9600);
    while (!Serial) { delay(100); }
    jetsonPacketSerial.setStream(&Serial);

    // Setup PacketSerial to handle communicating from Serial1
    Serial1.begin(HOLYBRO_BAUDRATE);
    while (!Serial1) { delay(100); }
    viconPacketSerial.setStream(&Serial1);
    viconPacketSerial.setPacketHandler(&onViconRecieved);

    // Start up SPI imu and initialize
    if (!bno.begin()) {
        if (DEBUG) { Serial.print("\nOoops, no BNO055 detected ... Check your wiring or I2C ADDR!\n"); };
        while (true);
    }
    // If necessary force calibration and wait until successful
    if (calibrateIMU(bno, FORCE_CALI)) {
        if (DEBUG) { Serial.println("\nSuccessfully Calibrated IMU\n"); };
    }
}

void loop() {
    viconPacketSerial.update();
    jetsonPacketSerial.update();

    bno.getEvent(&imu_event);
    if (DEBUG) { displaySensorReading(bno); }

    getImuInput(bno, IMU_input);
    sendImuMessage(jetsonPacketSerial, IMU_VICON_message);
    
    if (jetsonPacketSerial.overflow() || viconPacketSerial.overflow()) {
        digitalWrite(LED_PIN, HIGH);
    }
    else {
        digitalWrite(LED_PIN, LOW);
    }
}

/*
 * Relay the message over from Holybro to Jetson
 */
void onViconRecieved(const uint8_t *buffer, size_t size) {
    pb_istream_t stream = pb_istream_from_buffer(buffer, size);
    int status = pb_encode(&stream, messaging_VICON_fields, &VICON_measurement);
    if (!status) {
        Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream));
    }
}

/*
 * Send IMU protocol buff to jetson
 */
void sendMessage(PacketSerial &myPacketSerial, messaging_IMU_VICON &mes) {
    /* Create a stream that will write to our buffer. */
    pb_ostream_t stream = pb_ostream_from_buffer(imu_buffer, sizeof(imu_buffer));
    int status = pb_encode(&stream, messaging_IMU_fields, &input);
    if (!status) {
        Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream));
    }
    // Get the number of bytes written
    int message_length = stream.bytes_written;
    // Send to Jetson
    myPacketSerial.send(imu_buffer, message_length);
}
