#pragma once

#include <inttypes.h>

namespace rexquad {

struct PoseMsg {
    static constexpr uint8_t MsgID() { return 11; }
    float x;
    float y;
    float z;
    float qw;
    float qx;
    float qy;
    float qz;
};

void PoseToBytes(char* buf, const PoseMsg& pose);
bool PoseFromBytes(PoseMsg& pose, const char* buf);

}  // namespace rexquad