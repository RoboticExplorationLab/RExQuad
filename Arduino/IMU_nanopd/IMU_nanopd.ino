#include "IMU_helper.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PacketSerial.h>

#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

#define HOLYBRO_BAUDRATE (57600)

// #define DEBUG (true)
#define DEBUG (false)

#define MAX_FREQ (100)

// #define FORCE_CALI (true)
#define FORCE_CALI (false)

#define LED_PIN (13)

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
sensors_event_t imu_event;

// Build buffers and message types
uint8_t imu_buffer[256];
size_t imu_buffer_length = sizeof(imu_buffer);
uint8_t vicon_buffer[256];
size_t vicon_buffer_length = sizeof(vicon_buffer);

uint8_t imu_vicon_buffer[512];
size_t imu_vicon_buffer_length = sizeof(imu_vicon_buffer);

messaging_IMU IMU_input = messaging_IMU_init_zero;
messaging_VICON VICON_measurement = messaging_VICON_init_zero;
messaging_IMU_VICON IMU_VICON_message = {true, IMU_input, true, VICON_measurement};

// Initialize packet serial ports
PacketSerial jetsonPacketSerial;
PacketSerial viconPacketSerial;

void onViconRecieved(const uint8_t *buffer, size_t size);

// void sendMessage(PacketSerial &myPacketSerial, messaging_IMU &mes);
// void sendMessage(PacketSerial &myPacketSerial, messaging_VICON &mes);
void sendMessage(PacketSerial &myPacketSerial, messaging_IMU_VICON &mes);

long cnt = 0;
unsigned long last_time = micros(); 
unsigned long start_time = micros(); 



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

    randomSeed(42);
}

void loop() {
    start_time = micros();

    jetsonPacketSerial.update();
    viconPacketSerial.update();
    
    if (jetsonPacketSerial.overflow() || viconPacketSerial.overflow()) {
        digitalWrite(LED_PIN, HIGH);
    }
    else {
        digitalWrite(LED_PIN, LOW);
    }

    // Delay if we are going faster than the target frequency
    float target_loop_time = 1 / (double) MAX_FREQ;
    float actual_loop_time = ((micros() - start_time) * 1e-6);

    // Serial.print("Actual loop time: ");
    // Serial.print(actual_loop_time);
    // Serial.print(", Target loop time: ");
    // Serial.println(target_loop_time);

    if (actual_loop_time < target_loop_time) {
        int slow_down = (int) ((target_loop_time - actual_loop_time) * 1000.0);
        // Serial.print("Delayed by (Micro): ");
        // Serial.println(slow_down);
        delay(slow_down);
    }
}

/*
 * Relay the message over from Holybro to Jetson
 */
void onViconRecieved(const uint8_t *buffer, size_t size) {
    // Read in Vicon measurement
    pb_istream_t stream = pb_istream_from_buffer(buffer, size);
    int status = pb_decode(&stream, messaging_VICON_fields, &VICON_measurement);
    if (!status) {
        if (DEBUG) { Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream)); };
    }

    // Read in IMU measurement 
    bno.getEvent(&imu_event);
    
    if (DEBUG) { displaySensorReading(bno); }
    getImuInput(bno, IMU_input);

    // Send IMU/Vicon Message 
    IMU_VICON_message = {true, IMU_input, true, VICON_measurement};
    sendJetsonMessage(IMU_VICON_message);
}

/*
 * Relay the message over from Holybro to Jetson
 */
void sendJetsonMessage(messaging_IMU_VICON & mes) {
    /* Create a stream that will write to our buffer. */
    pb_ostream_t stream = pb_ostream_from_buffer(imu_vicon_buffer, imu_vicon_buffer_length);

    int status = pb_encode(&stream, messaging_IMU_VICON_fields, &mes);
    if (!status) {
        if (DEBUG) { Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream)); };
    }
    else {
        int message_length = stream.bytes_written;
        jetsonPacketSerial.send(imu_vicon_buffer, message_length);
    }

    // if (cnt % 100 == 0) {
    //     Serial.print("Loop frequency: ");
    //     Serial.println(100.0 / ((micros() - last_time) * 1e-6));
    //     last_time = micros();
    // }   
    // cnt += 1;
}