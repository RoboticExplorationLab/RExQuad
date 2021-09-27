#include <cstdint>
#include <functional>

#include "receiver.hpp"

struct LoRaViconReceiver
{
    rexlab::Pose<float> vicon_float;
    rexlab::Pose<int16_t> vicon_int16;
    size_t msg_size;
    uint8_t *buf;

    bool new_msg;
};

// Global Variable, Wasn't sure how to make use of the callback function using C++
LoRaViconReceiver global_receiver;

// void ConvertPoseToViconProtobuf(const rexlab::Pose<float> &pose, messaging_VICON *proto);

bool initialize_LoRaViconReceiver(uint8_t *buf, size_t msg_size)
{
    rexlab::Pose<float> vicon_float;
    rexlab::Pose<int16_t> vicon_int16;
    // Initalize global variable
    global_receiver.vicon_float = vicon_float;
    global_receiver.vicon_int16 = vicon_int16;
    global_receiver.msg_size = msg_size;
    global_receiver.buf = buf;
    global_receiver.new_msg = false;

    // Setup LoRa Communications
    LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
    if (!LoRa.begin(915E6))
    {
        Serial.println("Starting LoRa failed!");
        while (1)
        {
        };
    }
    // Optimal speed settings
    LoRa.setSpreadingFactor(6);
    LoRa.setSignalBandwidth(500E3);

    LoRa.onReceive(onLoRaReceive);
    LoRa.receive(global_receiver.msg_size);

    return true;
}

void onLoRaReceive(int packetSize)
{
    if (packetSize)
    {
        LoRa.readBytes(global_receiver.buf, global_receiver.msg_size);
        // Serial.write(global_receiver.buf, global_receiver.msg_size);
        global_receiver.vicon_int16 = *((rexlab::Pose<int16_t> *)global_receiver.buf);

        global_receiver.new_msg = true;
    }
}

bool hasLoRaRecieved()
{
    return global_receiver.new_msg;
}

void ConvertPoseToViconProtobuf(const rexlab::Pose<float> &pose, messaging_VICON *proto)
{
    proto->pos_x = pose.position_x;
    proto->pos_y = pose.position_y;
    proto->pos_z = pose.position_z;
    proto->quat_w = pose.quaternion_w;
    proto->quat_x = pose.quaternion_x;
    proto->quat_y = pose.quaternion_y;
    proto->quat_z = pose.quaternion_z;
    proto->time = static_cast<double>(pose.time_us) / 1e6;
}

void updateViconProto(messaging_VICON &vicon)
{
    ConvertPoseIntToFloat(global_receiver.vicon_int16, &global_receiver.vicon_float);
    ConvertPoseToViconProtobuf(global_receiver.vicon_float, &vicon);
    global_receiver.new_msg = false;
}

