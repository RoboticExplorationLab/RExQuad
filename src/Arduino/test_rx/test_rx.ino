#include <SPI.h>
#include <LoRa.h>

#define RF95_FREQ 915E6
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

void onLoRaReceive(int packetSize);

void setup()
{
    Serial.begin(9600);
    while (!Serial){
    };

    Serial.println("LoRa Receiver");

    LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
    if (!LoRa.begin(915E6))
    {
        Serial.println("Starting LoRa failed!");
        while (1){
        };
    }

    // Optimal speed settings
    LoRa.setSpreadingFactor(6);
    LoRa.setSignalBandwidth(500E3);

    LoRa.onReceive(onLoRaReceive);
    LoRa.receive(13);
    LoRa.enableCrc();
    // LoRa.receive(global_receiver.msg_size + 2); // Add on the 2 byte CRC suffix
}

void loop()
{
}

void onLoRaReceive(int packetSize)
{
    // received a packet
    Serial.print("Received packet '");

    // read packet
    for (int i = 0; i < packetSize; i++)
    {
        Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
}
