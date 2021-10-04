#ifndef motor_commands_hpp
#define motor_commands_hpp

#include "Arduino.h"
#include "Servo.h"

#define LED_PIN (13)

#define MAX_THROTLE (1832)
#define MIN_THROTLE (1148)

typedef struct _MOTOR_COMMANDS
{
    float front_left;
    float front_right;
    float back_right;
    float back_left;

    double time;
} MOTOR_COMMANDS;

typedef struct _MOTORS
{
    Servo front_left_esc;
    Servo front_right_esc;
    Servo back_right_esc;
    Servo back_left_esc;
} MOTORS;

void command_motors(MOTORS &motors, MOTOR_COMMANDS &command);

void arm(MOTORS &motors);

void calibrate(MOTORS &motors);

void kill(MOTORS &motors);

void print_command(MOTOR_COMMANDS &command);

void blink();

#endif