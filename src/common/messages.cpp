#include "messages.hpp"

#include "utils.hpp"

namespace rexquad {

void MeasurementMsgFromBytes(MeasurementMsg& msg, uint8_t* buf) {
  msg.x = bytestofloat(buf, 0 * 4);
  msg.y = bytestofloat(buf, 1 * 4);
  msg.z = bytestofloat(buf, 2 * 4);
  msg.qw = bytestofloat(buf, 3 * 4);
  msg.qx = bytestofloat(buf, 4 * 4);
  msg.qy = bytestofloat(buf, 5 * 4);
  msg.qz = bytestofloat(buf, 6 * 4);
  msg.ax = bytestofloat(buf, 7 * 4);
  msg.ay = bytestofloat(buf, 8 * 4);
  msg.az = bytestofloat(buf, 9 * 4);
  msg.wx = bytestofloat(buf, 10 * 4);
  msg.wy = bytestofloat(buf, 11 * 4);
  msg.wz = bytestofloat(buf, 12 * 4);
}

void MeasurementMsgToBytes(const MeasurementMsg& msg, uint8_t* buf) {
    floattobytes(buf, msg.x, 0 * 4);
    floattobytes(buf, msg.y, 1 * 4);
    floattobytes(buf, msg.z, 2 * 4);
    floattobytes(buf, msg.qw, 3 * 4);
    floattobytes(buf, msg.qx, 4 * 4);
    floattobytes(buf, msg.qy, 5 * 4);
    floattobytes(buf, msg.qz, 6 * 4);
    floattobytes(buf, msg.ax, 7 * 4);
    floattobytes(buf, msg.ay, 8 * 4);
    floattobytes(buf, msg.az, 9 * 4);
    floattobytes(buf, msg.wx, 10 * 4);
    floattobytes(buf, msg.wy, 11 * 4);
    floattobytes(buf, msg.wz, 12 * 4);
}


}  // rexquad