#include <SPI.h>
#include <LoRa.h>

#define RF95_FREQ 915E6
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

unsigned char str[] = "Hello world!";
size_t str_len = sizeof(str);

void setup()
{
    Serial.begin(9600);
    while (!Serial){
    };

    Serial.println("LoRa Sender");

    LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
    if (!LoRa.begin(915E6))
    {
        Serial.println("Starting LoRa failed!");
        while (1){
        };
    }
    // // Optimal speed settings
    LoRa.setSpreadingFactor(6);
    LoRa.setSignalBandwidth(500E3);
    LoRa.enableCrc();
}

void loop()
{
    Serial.println("Sending packet");

    // send packet
    LoRa.beginPacket(true);
    LoRa.write(str, str_len);
    LoRa.endPacket();

    delay(1000);
}