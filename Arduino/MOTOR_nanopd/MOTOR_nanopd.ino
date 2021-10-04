#include "PacketSerial.h"
#include "motor_commands.hpp"

// #define DEBUG           (true)
#define DEBUG           (false)

#define LED_PIN         (13)

#define FRONT_LEFT_PIN (9)
#define FRONT_RIGHT_PIN (10)
#define BACK_RIGHT_PIN (11)
#define BACK_LEFT_PIN (12)

// Define the Motor ESC structs
MOTORS motors;
MOTOR_COMMANDS command = {MIN_THROTLE, MIN_THROTLE, MIN_THROTLE, MIN_THROTLE, 0.0};

// Build buffers and message types
uint8_t motors_buffer[256];
size_t motors_buffer_length = sizeof(motors_buffer);

// Initialize packet serial ports
PacketSerial jetsonPacketSerial;

void onMotorsRecieved(const uint8_t *buffer, size_t size);


void setup() {
    pinMode(LED_PIN, OUTPUT);

    // Setup Motor ESC
    motors.front_left_esc.attach(FRONT_LEFT_PIN, MIN_THROTLE, MAX_THROTLE);
    motors.front_right_esc.attach(FRONT_RIGHT_PIN, MIN_THROTLE, MAX_THROTLE);
    motors.back_right_esc.attach(BACK_RIGHT_PIN, MIN_THROTLE, MAX_THROTLE);
    motors.back_left_esc.attach(BACK_LEFT_PIN, MIN_THROTLE, MAX_THROTLE);

    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(57600);
    while (!Serial)
    {
        delay(100);
    }
    Serial.println("Connected to Serial");

    jetsonPacketSerial.setStream(&Serial);
    jetsonPacketSerial.setPacketHandler(&onMotorsRecieved);

    calibrate(motors);
    Serial.println("Finsihed Calibration");

    // arm();
    // Serial.println("Armed");
}

void loop() {
    jetsonPacketSerial.update();

    // print_command(command);

    // command_motors(motors, command);
}

/*
 * Callback function called when a packet comes in over serial port
 */
void onMotorsRecieved(const uint8_t *buffer, size_t size) {
    if (size == sizeof(MOTOR_COMMANDS))
    {
        memcpy(&command, buffer, size);
    }
}