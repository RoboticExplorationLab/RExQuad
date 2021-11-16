#include "motor_commands.hpp"

#define FRONT_LEFT_PIN  (9)
#define FRONT_RIGHT_PIN (10)
#define BACK_RIGHT_PIN  (11)
#define BACK_LEFT_PIN   (12)

#define DEADMAN_MILLIS  (100)

// Define the Motor ESC structs
MOTORS motors;
MOTOR_COMMANDS command = MOTOR_COMMANDS_zero;
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
    Serial.println("Connected to Serial");

    // Setup Motor ESC
    delay(1000);
    motors = initialize_motors(&command, FRONT_LEFT_PIN, FRONT_RIGHT_PIN, BACK_RIGHT_PIN, BACK_LEFT_PIN);

    for (int i=0; i<10; i++)
    {
        blink();
    }
}

void loop()
{
    receiveJetsonMessage(command);
    deadman_switch(command);

    // print_command(command);
    command_motors(motors, command);
}

void receiveJetsonMessage(MOTOR_COMMANDS &command)
{
    int msg_bytes = sizeof(MOTOR_COMMANDS);

    if (Serial.available() == msg_bytes)
    {
        Serial.readBytes((char *)&command, msg_bytes);
        switch_led();
        last_time = millis();
    }
}

void deadman_switch(MOTOR_COMMANDS &command)
{
    if (abs(last_time - millis()) > DEADMAN_MILLIS)
    {
        command = MOTOR_COMMANDS_zero;
    }
}