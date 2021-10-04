#include "PacketSerial.h"
#include "motor_commands.hpp"

#define FRONT_LEFT_PIN  (9)
#define FRONT_RIGHT_PIN (10)
#define BACK_RIGHT_PIN  (11)
#define BACK_LEFT_PIN   (12)

// Define the Motor ESC structs
MOTORS motors;
MOTOR_COMMANDS command = initial_motor_commands_default;

// Build buffers and message types
uint8_t motors_buffer[256];
size_t motors_buffer_length = sizeof(motors_buffer);

// Initialize packet serial ports
PacketSerial jetsonPacketSerial;
void onMotorsRecieved(const uint8_t *buffer, size_t size);


void setup() {
    pinMode(LED_PIN, OUTPUT);

    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(115200);
    while (!Serial)
    {
        delay(100);
    }
    Serial.println("Connected to Serial");

    jetsonPacketSerial.setStream(&Serial);
    jetsonPacketSerial.setPacketHandler(&onMotorsRecieved);

    delay(1000);

    // Setup Motor ESC
    motors = initialize_motors(&command, FRONT_LEFT_PIN, FRONT_RIGHT_PIN, BACK_RIGHT_PIN, BACK_LEFT_PIN);

    for (int i=0; i<10; i++)
    {
        blink();    
    }
}

void loop() {
    jetsonPacketSerial.update();

//     print_command(command);
    command_motors(motors, command);
}

/*
 * Callback function called when a packet comes in over serial port
 */
void onMotorsRecieved(const uint8_t *buffer, size_t buffer_size) {
    if (buffer_size == sizeof(MOTOR_COMMANDS))
    {
        memcpy(&command, buffer, buffer_size);
        switch_led();
    }
}
