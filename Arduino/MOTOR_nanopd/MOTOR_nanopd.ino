#include "PacketSerial.h"

#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "motors_msg.pb.h"

// #define DEBUG           (true)
#define DEBUG           (false)

#define LED_PIN         (13)

#define FRONT_LEFT_PIN  (9);
#define FRONT_RIGHT_PIN (10);
#define BACK_RIGHT_PIN  (11);
#define BACK_LEFT_PIN   (12);


// Build buffers and message types
uint8_t motors_buffer[256];
size_t motors_buffer_length = sizeof(motors_buffer);

messaging_MOTORS MOTORS_input = messaging_MOTORS_init_zero;

// Initialize packet serial ports
PacketSerial jetsonPacketSerial;

void onMotorsRecieved(const uint8_t *buffer, size_t size);
void displayMessage(messaging_MOTORS &mes); 
void sendMessage(messaging_MOTORS &mes);


void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(FRONT_LEFT_PIN, OUTPUT);    
    pinMode(FRONT_RIGHT_PIN, OUTPUT);    
    pinMode(BACK_RIGHT_PIN, OUTPUT);    
    pinMode(BACK_LEFT_PIN, OUTPUT);    

    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(921600);
    while (!Serial) { delay(100); }
    jetsonPacketSerial.setStream(&Serial);
    jetsonPacketSerial.setPacketHandler(&onMotorsRecieved);

    randomSeed(42);
}

void loop() {
    jetsonPacketSerial.update();
    viconPacketSerial.update();
 
    if (DEBUG) { displayMessage(mes); }

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

/*
 * Relay the message over from Holybro to Jetson
 */
void sendMessage(messaging_MOTORS &mes) {
    /* Create a stream that will write to our buffer. */
    analogWrite(FRONT_LEFT_PIN, mes.front_left);
    analogWrite(FRONT_RIGHT_PIN, mes.front_right);
    analogWrite(BACK_RIGHT_PIN, mes.back_right);
    analogWrite(BACK_LEFT_PIN, mes.back_left);
}