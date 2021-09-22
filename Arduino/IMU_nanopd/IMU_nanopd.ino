#include "IMU_helper.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PacketSerial.h>

#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include "receiver.hpp"
#include "pose.hpp"

// Global Constants
#define HOLYBRO_BAUDRATE (57600)
#define DEBUG (false)
#define FORCE_CALI (false)
#define LED_PIN (13)
constexpr float rate_print_frequency = 1.0;  // frequency at which to print the averaged rate
constexpr HardwareSerial& HolybroRadio = Serial5;

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
sensors_event_t imu_event;

// Vicon
using ViconMsg = rexlab::Pose<int32_t>;
using Pose = rexlab::Pose<float>;

constexpr int BUFLEN = sizeof(ViconMsg);
ViconMsg vicon_pose_fixed_point;
Pose vicon_pose_float;
char* vicon_buffer = reinterpret_cast<char*>(&vicon_pose_fixed_point);
rexlab::SerialReceiver receiver(HolybroRadio, ViconMsg::MsgID());

/**
 * @brief Converts the internal Pose C++ type (with floating point data) to a Vicon 
 *        Protobuf type
 * 
 * @param pose   The floating point pose received over serial radio
 * @param proto  The vicon protobuf message to be modified
 */
void ConvertPoseToViconProtobuf(const Pose& pose, messaging_VICON* proto);


// Build buffers and message types
uint8_t imu_buffer[256];
size_t imu_buffer_length = sizeof(imu_buffer);
// uint8_t vicon_buffer[256];
// size_t vicon_buffer_length = sizeof(vicon_buffer);

uint8_t imu_vicon_buffer[512];
size_t imu_vicon_buffer_length = sizeof(imu_vicon_buffer);

// Protobuf types
messaging_IMU IMU_input = messaging_IMU_init_zero;
messaging_VICON VICON_measurement = messaging_VICON_init_zero;
messaging_IMU_VICON IMU_VICON_message = {true, IMU_input, true, VICON_measurement};

// Initialize packet serial ports
PacketSerial jetsonPacketSerial;
// PacketSerial viconPacketSerial;

// void onViconRecieved(const uint8_t *buffer, size_t size);

// void sendMessage(PacketSerial &myPacketSerial, messaging_IMU &mes);
// void sendMessage(PacketSerial &myPacketSerial, messaging_VICON &mes);
void sendMessage(PacketSerial &myPacketSerial, messaging_IMU_VICON &mes);

long cnt = 0;
unsigned long last_time = micros();
unsigned long start_time = micros();

// Startup
void setup() {
    pinMode(LED_PIN, OUTPUT);

    // Vicon Setup
    receiver.Init();
    receiver.EnableRatePrinting(rate_print_frequency);
    HolybroRadio.begin(HOLYBRO_BAUDRATE);

    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(9600);
    while (!Serial) { delay(100); }
    jetsonPacketSerial.setStream(&Serial);
    if (!HolybroRadio) { delay(100); }

    // Setup PacketSerial to handle communicating from Serial1
    // Serial1.begin(HOLYBRO_BAUDRATE);
    // while (!Serial1) { delay(100); }
    // viconPacketSerial.setStream(&Serial1);
    // viconPacketSerial.setPacketHandler(&onViconRecieved);

    // Start up SPI IMU and initialize
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

    // Read in Vicon Message (will block until timeout or receive)
    // TODO: make this non-blocking
    bool msg_received = receiver.Receive(vicon_buffer, BUFLEN);
    if (msg_received) {
        // Convert fixed-point message to floating point
        ConvertPoseIntToFloat(vicon_pose_fixed_point, &vicon_pose_float);

        // Convert pose type to Protobuf vicon message
        ConvertPoseToViconProtobuf(vicon_pose_float, &VICON_measurement);
    }

    // Read in IMU measurement
    bno.getEvent(&imu_event);
    getImuInput(bno, IMU_input);

    // Send IMU/Vicon Message
    // QUESTION: Do we need to re-assign the message fields if we're overwriting the data?
    IMU_VICON_message = {true, IMU_input, true, VICON_measurement};
    sendJetsonMessage(IMU_VICON_message);

    if (jetsonPacketSerial.overflow()) {
        digitalWrite(LED_PIN, HIGH);
    }
    else {
        digitalWrite(LED_PIN, LOW);
    }
}

// /*
//  * Relay the message over from Holybro to Jetson
//  */
// void onViconRecieved(const uint8_t *buffer, size_t size) {
//     // Read in Vicon measurement
//     pb_istream_t stream = pb_istream_from_buffer(buffer, size);
//     int status = pb_decode(&stream, messaging_VICON_fields, &VICON_measurement);
//     if (!status) {
//         if (DEBUG) { Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream)); };
//     }
// }

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
}

void ConvertPoseToViconProtobuf(const Pose& pose, messaging_VICON* proto) {
    proto->pos_x = pose.position_x;
    proto->pos_y = pose.position_y;
    proto->pos_z = pose.position_z;
    proto->quat_w = pose.quaternion_w;
    proto->quat_x = pose.quaternion_x;
    proto->quat_y = pose.quaternion_y;
    proto->quat_z = pose.quaternion_z;
    proto->time = static_cast<double>(pose.time_us) / 1e6;
}
