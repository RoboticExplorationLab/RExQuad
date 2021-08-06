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

void sendMessage(PacketSerial &myPacketSerial, messaging_IMU &mes);
void sendMessage(PacketSerial &myPacketSerial, messaging_VICON &mes);
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

    randomSeed(42);
}

void loop() {
    jetsonPacketSerial.update();
    viconPacketSerial.update();

    bno.getEvent(&imu_event);
    if (DEBUG) { displaySensorReading(bno); }

    getImuInput(bno, IMU_input);

    // mes.imu = IMU_input;
    // mes.vicon = VICON_measurement;
    IMU_VICON_message = {true, IMU_input, true, VICON_measurement};
    sendMessage(jetsonPacketSerial, IMU_VICON_message);

    // VICON_measurement = {1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    // sendMessage(jetsonPacketSerial, IMU_input);
    // sendMessage(jetsonPacketSerial, VICON_measurement);
    
    if (jetsonPacketSerial.overflow() || viconPacketSerial.overflow()) {
    // if (jetsonPacketSerial.overflow()) {
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
    int status = pb_decode(&stream, messaging_VICON_fields, &VICON_measurement);
    if (!status) {
        Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream));
    }
}

/*
 * Send IMU protocol buff to jetson
 */
void sendMessage(PacketSerial &myPacketSerial, messaging_IMU & mes) {
// void sendMessage(PacketSerial &myPacketSerial, messaging_IMU_VICON *mes) {
    /* Create a stream that will write to our buffer. */
    pb_ostream_t stream = pb_ostream_from_buffer(imu_buffer, imu_buffer_length);
    int status = pb_encode(&stream, messaging_IMU_fields, &mes);
    if (!status) {
        Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream));
    }
    Serial.println();    
    Serial.print(status);
    Serial.print(", Packed IMU message: ");
    Serial.println(stream.bytes_written);
}

void sendMessage(PacketSerial &myPacketSerial, messaging_VICON & mes) {
    /* Create a stream that will write to our buffer. */
    Serial.println();
    Serial.print(mes.pos_x);     Serial.print(", ");
    Serial.print(mes.pos_y);     Serial.print(", ");
    Serial.print(mes.pos_z);     Serial.print(", ");
    Serial.println();

    pb_ostream_t stream = pb_ostream_from_buffer(vicon_buffer, vicon_buffer_length);
    int status = pb_encode(&stream, messaging_VICON_fields, &mes);
    if (!status) {
        Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream));
    }
    Serial.println();
    Serial.print(status);
    Serial.print(", Packed VICON message: ");
    Serial.println(stream.bytes_written);
}

void sendMessage(PacketSerial &myPacketSerial, messaging_IMU_VICON & mes) {
    /* Create a stream that will write to our buffer. */
    pb_ostream_t stream = pb_ostream_from_buffer(imu_vicon_buffer, imu_vicon_buffer_length);

    // pb_encode_submessage(&stream, messaging_IMU_fields, &(mes.imu));
    // pb_encode_submessage(&stream, messaging_VICON_fields, &(mes.vicon));

    int status = pb_encode(&stream, messaging_IMU_VICON_fields, &mes);
    if (!status) {
        Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream));
    }
    Serial.println();
    Serial.print(status);
    Serial.print(", Packed VICON message: ");
    Serial.println(stream.bytes_written);
}



// void sendMessage(PacketSerial &myPacketSerial, messaging_IMU_VICON *mes) {
    // if (!pb_encode_submessage(&stream, messaging_IMU_fields, &(mes.imu))) {
    //     Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream));
    // }
    // else {
    //     if (!pb_encode_submessage(&stream, messaging_IMU_fields, &(mes.imu))) {
    //         Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream));
    //     }
    //     else {
    //         int status = pb_encode(&stream, messaging_IMU_VICON_fields, &mes);
    //         if (!status) {
    //             Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream));
    //         }

    //         Serial.println(status);
    //         Serial.println(PB_GET_ERROR(&stream));
    //         Serial.println(mes.imu.acc_x);

    //         // Get the number of bytes written
    //         int message_length = stream.bytes_written;

    //         Serial.println();
    //         Serial.println(message_length);
    //         Serial.println();

    //         // Send to Jetson
    //         myPacketSerial.send(imu_vicon_buffer, message_length);

    //     }
    // }  
// }

