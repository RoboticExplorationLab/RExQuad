void setup() {
  Serial.begin(128000);
}

char buf[100];

void loop() {
  int bytes_available = Serial.available();
  if (bytes_available) {
    Serial.readBytes(buf, bytes_available);
    Serial.write(buf, bytes_available);
  }
}