#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13
#define MSG_SIZE 4

char buf[100];

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  int bytes_available = Serial.available();
  if (bytes_available > 0) {
    Serial.readBytes(buf, bytes_available);
    Serial.write(buf, MSG_SIZE);
  }
}