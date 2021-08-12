#include "PacketSerial.h"
#include "Servo.h"

#include <pb_common.h>
#include <pb.h>
#include <pb_decode.h>
#include "motors_msg.pb.h"

// #define DEBUG           (true)
#define DEBUG           (false)

#define LED_PIN         (13)

#define FRONT_LEFT_PIN  (9)
#define FRONT_RIGHT_PIN (10)
#define BACK_RIGHT_PIN  (11)
#define BACK_LEFT_PIN   (12)

#define MAX_THROTLE     (1832)
#define MIN_THROTLE     (1148)

// Define the Motor ESC structs
Servo front_left_esc, front_right_esc, back_right_esc, back_left_esc;
bool light_on = true;

// Build buffers and message types
uint8_t motors_buffer[256];
size_t motors_buffer_length = sizeof(motors_buffer);
messaging_MOTORS MOTORS_input = {MIN_THROTLE, MIN_THROTLE, MIN_THROTLE, MIN_THROTLE, 0.0};

// Initialize packet serial ports
PacketSerial jetsonPacketSerial;

unsigned long last_time = millis();

void onMotorsRecieved(const uint8_t *buffer, size_t size);
void sendMessage(messaging_MOTORS &mes);
void arm();
void calibrate();
void kill();
void blink();

void setup() {
    pinMode(LED_PIN, OUTPUT);

    // Setup Motor ESC
    front_left_esc.attach(FRONT_LEFT_PIN, MIN_THROTLE, MAX_THROTLE);
    front_right_esc.attach(FRONT_RIGHT_PIN, MIN_THROTLE, MAX_THROTLE);
    back_right_esc.attach(BACK_RIGHT_PIN, MIN_THROTLE, MAX_THROTLE);
    back_left_esc.attach(BACK_LEFT_PIN, MIN_THROTLE, MAX_THROTLE);

    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(57600);
    while (!Serial) { delay(100); }
    Serial.println("Connected to Serial");

    jetsonPacketSerial.setStream(&Serial);
    jetsonPacketSerial.setPacketHandler(&onMotorsRecieved);

    calibration();
    Serial.println("Finsihed Calibration");

    // arm();
    // Serial.println("Armed");
}

void loop() {
    jetsonPacketSerial.update();

    sendMessage(MOTORS_input);

    if ((last_time - millis()) > 500) {
        kill();
    }
}

/*
 * Relay the message over from Holybro to Jetson
 */
void onMotorsRecieved(const uint8_t *buffer, size_t size) {
    pb_istream_t stream = pb_istream_from_buffer(buffer, size);
    int status = pb_decode(&stream, messaging_MOTORS_fields, &MOTORS_input);
    if (!status) {
        Serial.printf("\nDecoding failed: %s\n", PB_GET_ERROR(&stream));
    }
    else {
        if (light_on) {
            digitalWrite(LED_PIN, LOW);
        }
        else {
            digitalWrite(LED_PIN, HIGH);
        }
        light_on = !light_on;
        last_time = millis();
    }
}

void sendMessage(messaging_MOTORS &mes) {
    if (MIN_THROTLE <= mes.front_left && mes.front_left <= MAX_THROTLE) {
        front_left_esc.writeMicroseconds((int) mes.front_left);
    }
    if (MIN_THROTLE <= mes.front_right && mes.front_right <= MAX_THROTLE) {
        front_right_esc.writeMicroseconds((int)mes.front_right);
    }
    if (MIN_THROTLE <= mes.back_right && mes.back_right <= MAX_THROTLE) {
        back_right_esc.writeMicroseconds((int) mes.back_right);
    }
    if (MIN_THROTLE <= mes.back_left && mes.back_left <= MAX_THROTLE) {
        back_left_esc.writeMicroseconds((int) mes.back_left);
    }
}

void calibration() {
    // front_left_esc.writeMicroseconds(MAX_THROTLE);
    // front_right_esc.writeMicroseconds(MAX_THROTLE);
    // back_right_esc.writeMicroseconds(MAX_THROTLE);
    // back_left_esc.writeMicroseconds(MAX_THROTLE);
    // delay(3000);

    // front_left_esc.writeMicroseconds(MIN_THROTLE);
    // front_right_esc.writeMicroseconds(MIN_THROTLE);
    // back_right_esc.writeMicroseconds(MIN_THROTLE);
    // back_left_esc.writeMicroseconds(MIN_THROTLE);
    // delay(8000);

    front_left_esc.writeMicroseconds(MAX_THROTLE);
    delay(3000);
    //Serial.println("throttle down 1");
    front_left_esc.writeMicroseconds(MIN_THROTLE);
    delay(3000);

    //Serial.println("throttle up 2") ;
    front_right_esc.writeMicroseconds(MAX_THROTLE);
    delay(3000);
    //Serial.println("throttle down 2");
    front_right_esc.writeMicroseconds(MIN_THROTLE);
    delay(3000);

    //Serial.println("throttle up 3") ;
    back_right_esc.writeMicroseconds(MAX_THROTLE);
    delay(3000);
    //Serial.println("throttle down 3");
    back_right_esc.writeMicroseconds(MIN_THROTLE);
    delay(3000);

    //Serial.println("throttle up 4") ;
    back_left_esc.writeMicroseconds(MAX_THROTLE);
    delay(3000);
    //Serial.println("throttle down 4");
    back_left_esc.writeMicroseconds(MIN_THROTLE);
    delay(8000);

    for (int i = 0; i < 2; i++) {
        blink();
    }
}

void arm() {
    front_left_esc.writeMicroseconds(MAX_THROTLE);
    front_right_esc.writeMicroseconds(MAX_THROTLE);
    back_right_esc.writeMicroseconds(MAX_THROTLE);
    back_left_esc.writeMicroseconds(MAX_THROTLE);
    delay(2000);

    front_left_esc.writeMicroseconds(MIN_THROTLE + 200);
    front_right_esc.writeMicroseconds(MIN_THROTLE + 200);
    back_right_esc.writeMicroseconds(MIN_THROTLE + 200);
    back_left_esc.writeMicroseconds(MIN_THROTLE + 200);
    delay(2000);

    for (int i = 0; i < 3; i++) {
        blink();
    }
}

void kill() {
    front_left_esc.writeMicroseconds(MIN_THROTLE);
    front_right_esc.writeMicroseconds(MIN_THROTLE);
    back_right_esc.writeMicroseconds(MIN_THROTLE);
    back_left_esc.writeMicroseconds(MIN_THROTLE);
}

void blink() {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(50);
}