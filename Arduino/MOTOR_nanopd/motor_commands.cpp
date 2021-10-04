#include "motor_commands.hpp"

bool led_state = false;

void switch_led()
{
    if (led_state)
    {
        digitalWrite(LED_PIN, LOW);
    }
    else
    {
        digitalWrite(LED_PIN, HIGH);
    }
    led_state = !led_state;
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

    switch_led();
}

/*
 * Calibrate the ESCs
 */
void calibrate(MOTORS &motors)
{
    motors.front_left_esc.writeMicroseconds(MAX_THROTLE);
    delay(3000);
    //Serial.println("throttle down 1");
    motors.front_left_esc.writeMicroseconds(MIN_THROTLE);
    delay(3000);

    //Serial.println("throttle up 2") ;
    motors.front_right_esc.writeMicroseconds(MAX_THROTLE);
    delay(3000);
    //Serial.println("throttle down 2");
    motors.front_right_esc.writeMicroseconds(MIN_THROTLE);
    delay(3000);

    //Serial.println("throttle up 3") ;
    motors.back_right_esc.writeMicroseconds(MAX_THROTLE);
    delay(3000);
    //Serial.println("throttle down 3");
    motors.back_right_esc.writeMicroseconds(MIN_THROTLE);
    delay(3000);

    //Serial.println("throttle up 4") ;
    motors.back_left_esc.writeMicroseconds(MAX_THROTLE);
    delay(3000);
    //Serial.println("throttle down 4");
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

void print_command(MOTOR_COMMANDS &command)
{
    Serial.printf("Commanded: [%f, %f, %f, %f]\n",
                  command.front_left,
                  command.front_right,
                  command.back_right,
                  command.back_left);
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
