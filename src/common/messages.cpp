#include "messages.hpp"

#include "utils.hpp"

namespace rexquad {

void MeasurementMsgFromBytes(MeasurementMsg& msg, uint8_t* buf, int off) {
  msg.x = bytestofloat(buf, 0 * 4 + off);
  msg.y = bytestofloat(buf, 1 * 4 + off);
  msg.z = bytestofloat(buf, 2 * 4 + off);
  msg.qw = bytestofloat(buf, 3 * 4 + off);
  msg.qx = bytestofloat(buf, 4 * 4 + off);
  msg.qy = bytestofloat(buf, 5 * 4 + off);
  msg.qz = bytestofloat(buf, 6 * 4 + off);
  msg.ax = bytestofloat(buf, 7 * 4 + off);
  msg.ay = bytestofloat(buf, 8 * 4 + off);
  msg.az = bytestofloat(buf, 9 * 4 + off);
  msg.wx = bytestofloat(buf, 10 * 4 + off);
  msg.wy = bytestofloat(buf, 11 * 4 + off);
  msg.wz = bytestofloat(buf, 12 * 4 + off);
}

void MeasurementMsgToBytes(const MeasurementMsg& msg, uint8_t* buf, int off) {
  floattobytes(buf, msg.x, 0 * 4 + off);
  floattobytes(buf, msg.y, 1 * 4 + off);
  floattobytes(buf, msg.z, 2 * 4 + off);
  floattobytes(buf, msg.qw, 3 * 4 + off);
  floattobytes(buf, msg.qx, 4 * 4 + off);
  floattobytes(buf, msg.qy, 5 * 4 + off);
  floattobytes(buf, msg.qz, 6 * 4 + off);
  floattobytes(buf, msg.ax, 7 * 4 + off);
  floattobytes(buf, msg.ay, 8 * 4 + off);
  floattobytes(buf, msg.az, 9 * 4 + off);
  floattobytes(buf, msg.wx, 10 * 4 + off);
  floattobytes(buf, msg.wy, 11 * 4 + off);
  floattobytes(buf, msg.wz, 12 * 4 + off);
}

void ControlMsgFromBytes(ControlMsg& msg, uint8_t* buf, int off) {
  msg.data[0] = bytestofloat(buf, 0 * 4 + off);
  msg.data[1] = bytestofloat(buf, 1 * 4 + off);
  msg.data[2] = bytestofloat(buf, 2 * 4 + off);
  msg.data[3] = bytestofloat(buf, 3 * 4 + off);
}

void ControlMsgToBytes(const ControlMsg& msg, uint8_t* buf, int off) {
  floattobytes(buf, msg.data[0], 0 * 4 + off);
  floattobytes(buf, msg.data[1], 1 * 4 + off);
  floattobytes(buf, msg.data[2], 2 * 4 + off);
  floattobytes(buf, msg.data[3], 3 * 4 + off);
}

}  // namespace rexquad