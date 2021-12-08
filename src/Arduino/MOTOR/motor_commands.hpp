#ifndef motor_commands_hpp
#define motor_commands_hpp

#include <Arduino.h>
#include <Servo.h>

#define LED_PIN     (13)
#define MAX_THROTLE (1832.0)
#define MIN_THROTLE (1148.0)

typedef struct _MOTOR_COMMANDS
{
    float front_left;
    float front_right;
    float back_right;
    float back_left;
} MOTOR_COMMANDS;

#define MOTOR_COMMANDS_zero                                          \
    {                                                           \
        MIN_THROTLE, MIN_THROTLE, MIN_THROTLE, MIN_THROTLE \
    }

typedef struct _MOTORS
{
    Servo front_left_esc;
    Servo front_right_esc;
    Servo back_right_esc;
    Servo back_left_esc;
} MOTORS;

MOTORS initialize_motors(MOTOR_COMMANDS * command,
                         int front_left_pin,
                         int front_right_pin,
                         int back_right_pin,
                         int back_left_pin);

bool valid_command(MOTOR_COMMANDS &command);

void command_motors(MOTORS &motors, MOTOR_COMMANDS &command);

void arm(MOTORS &motors);

void calibrate(MOTORS &motors);

void kill(MOTORS &motors);

void print_command(MOTOR_COMMANDS command);

void blink();

void switch_led();

#endif
