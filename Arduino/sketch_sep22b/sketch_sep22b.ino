#include <SPI.h>
#include <LoRa.h>


#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3


int cnt = 0;
int start_time = micros();
int end_time = micros();


void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");
  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
//  LoRa.setSPIFrequency(12000000);
  
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  LoRa.setSpreadingFactor(6);
  LoRa.setSignalBandwidth(500E3);
//  LoRa.enableCrc();
//  LoRa.setPreambleLength(6);
//  LoRa.setGain(0);
}

void loop() {
  // try to parse packet
//  int packetSize = LoRa.parsePacket();
  int packetSize = LoRa.parsePacket(30);
  
  if (packetSize) {
    // received a packet
//    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
        LoRa.read();
//      Serial.print((char) LoRa.read());
    }

    // print RSSI of packet
//    Serial.print("' with RSSI ");
//    Serial.println(LoRa.packetRssi());

    if (!(cnt % 100)) {
        end_time = micros();
        Serial.print("Running at ~");
        Serial.print(100 / ((end_time - start_time) * 1e-6));
        Serial.println("Hz");
        start_time = micros();
    }
    cnt ++;
  }
}
