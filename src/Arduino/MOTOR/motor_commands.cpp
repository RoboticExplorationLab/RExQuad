#include "motor_commands.hpp"

bool led_state = false;

MOTORS initialize_motors(MOTOR_COMMANDS * command, int front_left_pin, int front_right_pin, int back_right_pin, int back_left_pin)
{
    MOTORS quad_motors;

    quad_motors.front_left_esc.attach(front_left_pin, MIN_THROTLE, MAX_THROTLE);
    quad_motors.front_right_esc.attach(front_right_pin, MIN_THROTLE, MAX_THROTLE);
    quad_motors.back_right_esc.attach(back_right_pin, MIN_THROTLE, MAX_THROTLE);
    quad_motors.back_left_esc.attach(back_left_pin, MIN_THROTLE, MAX_THROTLE);

    delay(1000);
    calibrate(quad_motors);
    Serial.println("Finsihed Calibration");

    // Set the command variable passed in to the MIN_THROTLE stick position
    *command = MOTOR_COMMANDS_zero;

    return quad_motors;
}

void command_motors(MOTORS &motors, MOTOR_COMMANDS &command)
{
    if (MIN_THROTLE <= command.front_left && command.front_left <= MAX_THROTLE)
    {
        motors.front_left_esc.writeMicroseconds((int)command.front_left);
    }
    if (MIN_THROTLE <= command.front_right && command.front_right <= MAX_THROTLE)
    {
        motors.front_right_esc.writeMicroseconds((int)command.front_right);
    }
    if (MIN_THROTLE <= command.back_right && command.back_right <= MAX_THROTLE)
    {
        motors.back_right_esc.writeMicroseconds((int)command.back_right);
    }
    if (MIN_THROTLE <= command.back_left && command.back_left <= MAX_THROTLE)
    {
        motors.back_left_esc.writeMicroseconds((int)command.back_left);
    }
}

/*
 * Calibrate the ESCs
 */
void calibrate(MOTORS &motors)
{
    motors.front_left_esc.writeMicroseconds(MAX_THROTLE);
    motors.front_right_esc.writeMicroseconds(MAX_THROTLE);
    motors.back_right_esc.writeMicroseconds(MAX_THROTLE);
    motors.back_left_esc.writeMicroseconds(MAX_THROTLE);
    delay(7000);

    //Serial.println("throttle down 1");
    motors.front_left_esc.writeMicroseconds(MIN_THROTLE);
    motors.front_right_esc.writeMicroseconds(MIN_THROTLE);
    motors.back_right_esc.writeMicroseconds(MIN_THROTLE);
    motors.back_left_esc.writeMicroseconds(MIN_THROTLE);
    delay(8000);

    for (int i = 0; i < 2; i++)
    {
        blink();
    }
}

/*
 * Arm the ESCs
 */
void arm(MOTORS &motors)
{
    motors.front_left_esc.writeMicroseconds(MAX_THROTLE);
    motors.front_right_esc.writeMicroseconds(MAX_THROTLE);
    motors.back_right_esc.writeMicroseconds(MAX_THROTLE);
    motors.back_left_esc.writeMicroseconds(MAX_THROTLE);
    delay(2000);

    motors.front_left_esc.writeMicroseconds(MIN_THROTLE + 200);
    motors.front_right_esc.writeMicroseconds(MIN_THROTLE + 200);
    motors.back_right_esc.writeMicroseconds(MIN_THROTLE + 200);
    motors.back_left_esc.writeMicroseconds(MIN_THROTLE + 200);
    delay(2000);

    for (int i = 0; i < 3; i++)
    {
        blink();
    }
}

/*
 * Kill the motors. This function is called if no message has been
 * heard in more than 0.5 seconds.
 */
void kill(MOTORS &motors)
{
    motors.front_left_esc.writeMicroseconds(MIN_THROTLE);
    motors.front_right_esc.writeMicroseconds(MIN_THROTLE);
    motors.back_right_esc.writeMicroseconds(MIN_THROTLE);
    motors.back_left_esc.writeMicroseconds(MIN_THROTLE);
}

void print_command(MOTOR_COMMANDS command)
{
    Serial.print(command.front_left); Serial.print(", ");
    Serial.print(command.front_right); Serial.print(", ");
    Serial.print(command.back_right); Serial.print(", ");
    Serial.print(command.back_left); Serial.print("\n");
}

/*
 * Blink LED 1 time
 */
void blink()
{
    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(50);
}


void switch_led()
{
    if (led_state)
    {
        digitalWrite(LED_PIN, LOW);
        led_state = false;
    }
    else
    {
        digitalWrite(LED_PIN, HIGH);
        led_state = true;
    }
}
