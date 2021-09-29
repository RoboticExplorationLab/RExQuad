#include <cstdint>
#include <functional>

#include <SPI.h>
#include <LoRa.h>

#include "pose.hpp"

#include "vicon_msg.pb.h"

#define RF95_FREQ 915.0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3


bool initialize_LoRaViconReceiver(uint8_t *buf, size_t msg_size);

void onLoRaReceive(int packetSize);

bool hasLoRaRecieved();

void updateViconProto(messaging_VICON &vicon);
