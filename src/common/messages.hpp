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

void MeasurementMsgFromBytes(MeasurementMsg& msg, uint8_t* buf, int off = 0);
void MeasurementMsgToBytes(const MeasurementMsg& msg, uint8_t* buf, int off = 0);

struct ControlMsg {
  float data[4];
};
void ControlMsgFromBytes(ControlMsg& msg, uint8_t* buf, int off = 0);
void ControlMsgToBytes(const ControlMsg& msg, uint8_t* buf, int off = 0);


}  // rexquad