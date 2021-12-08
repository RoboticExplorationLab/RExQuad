#include "motor_commands.hpp"
#include "crc8.h"

#define FRONT_LEFT_PIN  (9)
#define FRONT_RIGHT_PIN (10)
#define BACK_RIGHT_PIN  (11)
#define BACK_LEFT_PIN   (12)

#define DEADMAN_MILLIS  (1000)

// Define the Motor ESC structs
MOTORS motors;
MOTOR_COMMANDS command = MOTOR_COMMANDS_zero;
uint8_t motor_command_buffer[256];
CRC8_PARAMS crc8_params = DEFAULT_CRC8_PARAMS;
unsigned long last_time = millis();

void receiveJetsonMessage(MOTOR_COMMANDS &command);
void deadman_switch(MOTOR_COMMANDS &command);

void setup()
{
    pinMode(LED_PIN, OUTPUT);

    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(115200);
    while (!Serial)
    {
        delay(100);
    }
    for (int i = 0; i < 5; i++)
    {
        blink();
    }

    // Setup Motor ESC
    delay(1000);
    motors = initialize_motors(&command, FRONT_LEFT_PIN, FRONT_RIGHT_PIN, BACK_RIGHT_PIN, BACK_LEFT_PIN);
    delay(1000);
}

void loop()
{
    receiveJetsonMessage(command);
    // print_command(command);

    command_motors(motors, command);

    // deadman_switch(command);
}

void receiveJetsonMessage(MOTOR_COMMANDS &command)
{
    int msg_bytes = sizeof(MOTOR_COMMANDS);
    int bytes_read = Serial.available();

    // Bytes received should be message size plus 1 byte CRC flag
    if (bytes_read == msg_bytes+1)
    {
        // Read in all available bytes
        Serial.readBytes((char *)motor_command_buffer, bytes_read);

        // CRC byte should be last byte read
        uint8_t crc8_byte_recv = motor_command_buffer[bytes_read - 1];
        // Compute CRC value of remaining bytes
        uint8_t crc8_byte_comp = crc8(crc8_params, motor_command_buffer, bytes_read - 1);

        // Check if computed CRC matches received CRC
        if (crc8_byte_comp == crc8_byte_recv)
        {
            // Copy the message from the temporary buffer into the motor command struct
            MOTOR_COMMANDS tmp = MOTOR_COMMANDS_zero;
            memcpy(&tmp, motor_command_buffer, msg_bytes);

            // Makesure the message was decoded sensibly
            if (valid_command(tmp))
            {
                command = tmp;
                // Switch LED to indicate message was received
                switch_led();
                last_time = millis();
            }
        }
    }
    else {
        while (Serial.available())
        {
            Serial.read(); // if yes, read it
        }
    }
}

// If you haven't heard a message in DEADMAN_MILLIS milliseconds then
// zero out the motor commands and switch the indicator LED to high
void deadman_switch(MOTOR_COMMANDS &command)
{
    if (abs(last_time - millis()) > DEADMAN_MILLIS)
    {
        command = MOTOR_COMMANDS_zero;
    }
}