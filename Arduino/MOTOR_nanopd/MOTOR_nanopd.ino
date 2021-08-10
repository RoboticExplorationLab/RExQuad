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

Servo front_left_esc, front_right_esc, back_right_esc, back_left_esc;

// Build buffers and message types
uint8_t motors_buffer[256];
size_t motors_buffer_length = sizeof(motors_buffer);

messaging_MOTORS MOTORS_input = messaging_MOTORS_init_zero;

// Initialize packet serial ports
PacketSerial jetsonPacketSerial;

void onMotorsRecieved(const uint8_t *buffer, size_t size);
void displayMessage(messaging_MOTORS &mes);
void sendMessage(messaging_MOTORS &mes);
void arm();
void calibrate();


void setup() {
    pinMode(LED_PIN, OUTPUT);

    // Setup Motor ESC
    pinMode(FRONT_LEFT_PIN, OUTPUT);
    pinMode(FRONT_RIGHT_PIN, OUTPUT);
    pinMode(BACK_RIGHT_PIN, OUTPUT);
    pinMode(BACK_LEFT_PIN, OUTPUT);
    front_left_esc.attach(FRONT_LEFT_PIN, MAX_THROTLE, MIN_THROTLE);
    front_right_esc.attach(FRONT_RIGHT_PIN, MAX_THROTLE, MIN_THROTLE);
    back_right_esc.attach(BACK_RIGHT_PIN, MAX_THROTLE, MIN_THROTLE);
    back_left_esc.attach(BACK_LEFT_PIN, MAX_THROTLE, MIN_THROTLE);

    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(115200);
    while (!Serial) { delay(100); }
    jetsonPacketSerial.setStream(&Serial);
    jetsonPacketSerial.setPacketHandler(&onMotorsRecieved);

    arm();
}

void loop() {
    jetsonPacketSerial.update();

    if (DEBUG) { displayMessage(MOTORS_input); }

    if (jetsonPacketSerial.overflow()) {
        digitalWrite(LED_PIN, HIGH);
    }
    else {
        digitalWrite(LED_PIN, LOW);
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
}

void displayMessage(messaging_MOTORS &mes) {
    Serial.println();
    Serial.print("Front Left: "); Serial.print(mes.front_left);
    Serial.print("Front Right: "); Serial.println(mes.front_right);
    Serial.print("Back Right: "); Serial.println(mes.back_right);
    Serial.print("Back Left: "); Serial.println(mes.back_left);
    Serial.println();
}

void sendMessage(messaging_MOTORS &mes) {
    /* Create a stream that will write to our buffer. */
    if (MIN_THROTLE < mes.front_left && mes.front_left < MAX_THROTLE) {
        front_left_esc.writeMicroseconds(MAX_THROTLE);
    }
    if (MIN_THROTLE < mes.front_right && mes.front_right < MAX_THROTLE) {
        front_right_esc.writeMicroseconds(mes.front_left);
    }
    if (MIN_THROTLE < mes.back_right && mes.back_right < MAX_THROTLE) {
        back_right_esc.writeMicroseconds(mes.back_right);
    }
    if (MIN_THROTLE < mes.back_left && mes.back_left < MAX_THROTLE) {
        back_left_esc.writeMicroseconds(mes.back_left);
    }
}

void arm() {
    front_left_esc.writeMicroseconds(MAX_THROTLE);
    front_right_esc.writeMicroseconds(MAX_THROTLE);
    back_right_esc.writeMicroseconds(MAX_THROTLE);
    back_left_esc.writeMicroseconds(MAX_THROTLE);
    delay(2000);
    front_left_esc.writeMicroseconds(MIN_THROTLE+200);
    front_right_esc.writeMicroseconds(MIN_THROTLE+200);
    back_right_esc.writeMicroseconds(MIN_THROTLE+200);
    back_left_esc.writeMicroseconds(MIN_THROTLE+200);
    delay(2000);
}

void calibrate() {
}

