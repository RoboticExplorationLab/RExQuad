#pragma once

#include <inttypes.h>

namespace rexquad {

struct MeasurementMsg {
  float x;
  float y;
  float z;
  float qw;
  float qx;
  float qy;
  float qz;
  float ax;
  float ay;
  float az;
  float wx;
  float wy;
  float wz;
};

void MeasurementMsgFromBytes(MeasurementMsg& msg, uint8_t* buf);
void MeasurementMsgToBytes(const MeasurementMsg& msg, uint8_t* buf);


}  // rexquad