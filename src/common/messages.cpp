#include "messages.hpp"

#include "utils.hpp"

namespace rexquad {

bool MeasurementMsgFromBytes(MeasurementMsg& msg, uint8_t* buf, int off) {
  const uint8_t msgid = buf[off];
  ++off;
  msg.x = bytestofloat(buf, 0 * 4 + off);
  msg.y = bytestofloat(buf, 1 * 4 + off);
  msg.z = bytestofloat(buf, 2 * 4 + off);
  msg.qw = bytestofloat(buf, 3 * 4 + off);
  msg.qx = bytestofloat(buf, 4 * 4 + off);
  msg.qy = bytestofloat(buf, 5 * 4 + off);
  msg.qz = bytestofloat(buf, 6 * 4 + off);
  msg.vx = bytestofloat(buf, 7 * 4 + off);
  msg.vy = bytestofloat(buf, 8 * 4 + off);
  msg.vz = bytestofloat(buf, 9 * 4 + off);
  msg.ax = bytestofloat(buf, 10 * 4 + off);
  msg.ay = bytestofloat(buf, 11 * 4 + off);
  msg.az = bytestofloat(buf, 12 * 4 + off);
  msg.wx = bytestofloat(buf, 13 * 4 + off);
  msg.wy = bytestofloat(buf, 14 * 4 + off);
  msg.wz = bytestofloat(buf, 15 * 4 + off);
  return msgid == MeasurementMsg::MsgID;
}

void MeasurementMsgToBytes(const MeasurementMsg& msg, uint8_t* buf, int off) {
  buf[off] = MeasurementMsg::MsgID;
  ++off;
  floattobytes(buf, msg.x, 0 * 4 + off);
  floattobytes(buf, msg.y, 1 * 4 + off);
  floattobytes(buf, msg.z, 2 * 4 + off);
  floattobytes(buf, msg.qw, 3 * 4 + off);
  floattobytes(buf, msg.qx, 4 * 4 + off);
  floattobytes(buf, msg.qy, 5 * 4 + off);
  floattobytes(buf, msg.qz, 6 * 4 + off);
  floattobytes(buf, msg.vx, 7 * 4 + off);
  floattobytes(buf, msg.vy, 8 * 4 + off);
  floattobytes(buf, msg.vz, 9 * 4 + off);
  floattobytes(buf, msg.ax, 10 * 4 + off);
  floattobytes(buf, msg.ay, 11 * 4 + off);
  floattobytes(buf, msg.az, 12 * 4 + off);
  floattobytes(buf, msg.wx, 13 * 4 + off);
  floattobytes(buf, msg.wy, 14 * 4 + off);
  floattobytes(buf, msg.wz, 15 * 4 + off);
}

bool ControlMsgFromBytes(ControlMsg& msg, uint8_t* buf, int off) {
  const uint8_t msgid = buf[off];
  ++off;
  msg.data[0] = bytestofloat(buf, 0 * 4 + off);
  msg.data[1] = bytestofloat(buf, 1 * 4 + off);
  msg.data[2] = bytestofloat(buf, 2 * 4 + off);
  msg.data[3] = bytestofloat(buf, 3 * 4 + off);
  return msgid == ControlMsg::MsgID;
}

void ControlMsgToBytes(const ControlMsg& msg, uint8_t* buf, int off) {
  buf[off] = ControlMsg::MsgID;
  ++off;
  floattobytes(buf, msg.data[0], 0 * 4 + off);
  floattobytes(buf, msg.data[1], 1 * 4 + off);
  floattobytes(buf, msg.data[2], 2 * 4 + off);
  floattobytes(buf, msg.data[3], 3 * 4 + off);
}

bool StateControlMsgFromBytes(StateControlMsg& msg, uint8_t* buf, int off) {
  const uint8_t msgid = buf[off];
  ++off;
  msg.x = bytestofloat(buf, 0 * 4 + off);
  msg.y = bytestofloat(buf, 1 * 4 + off);
  msg.z = bytestofloat(buf, 2 * 4 + off);
  msg.qw = bytestofloat(buf, 3 * 4 + off);
  msg.qx = bytestofloat(buf, 4 * 4 + off);
  msg.qy = bytestofloat(buf, 5 * 4 + off);
  msg.qz = bytestofloat(buf, 6 * 4 + off);
  msg.vx = bytestofloat(buf, 7 * 4 + off);
  msg.vy = bytestofloat(buf, 8 * 4 + off);
  msg.vz = bytestofloat(buf, 9 * 4 + off);
  msg.wx = bytestofloat(buf, 10 * 4 + off);
  msg.wy = bytestofloat(buf, 11 * 4 + off);
  msg.wz = bytestofloat(buf, 12 * 4 + off);
  msg.u[0] = bytestofloat(buf, 13 * 4 + off);
  msg.u[1] = bytestofloat(buf, 14 * 4 + off);
  msg.u[2] = bytestofloat(buf, 15 * 4 + off);
  msg.u[3] = bytestofloat(buf, 16 * 4 + off);
  return msgid == StateControlMsg::MsgID;
}

void StateControlMsgToBytes(const StateControlMsg& msg, uint8_t* buf, int off) {
  buf[off] = StateControlMsg::MsgID;
  ++off;
  floattobytes(buf, msg.x, 0 * 4 + off);
  floattobytes(buf, msg.y, 1 * 4 + off);
  floattobytes(buf, msg.z, 2 * 4 + off);
  floattobytes(buf, msg.qw, 3 * 4 + off);
  floattobytes(buf, msg.qx, 4 * 4 + off);
  floattobytes(buf, msg.qy, 5 * 4 + off);
  floattobytes(buf, msg.qz, 6 * 4 + off);
  floattobytes(buf, msg.vx, 7 * 4 + off);
  floattobytes(buf, msg.vy, 8 * 4 + off);
  floattobytes(buf, msg.vz, 9 * 4 + off);
  floattobytes(buf, msg.wx, 10 * 4 + off);
  floattobytes(buf, msg.wy, 11 * 4 + off);
  floattobytes(buf, msg.wz, 12 * 4 + off);
  floattobytes(buf, msg.u[0], 13 * 4 + off);
  floattobytes(buf, msg.u[1], 14 * 4 + off);
  floattobytes(buf, msg.u[2], 15 * 4 + off);
  floattobytes(buf, msg.u[3], 16 * 4 + off);
}

bool StateMsgFromBytes(StateMsg& msg, uint8_t* buf, int off) {
  const uint8_t msgid = buf[off];
  ++off;
  msg.x = bytestofloat(buf, 0 * 4 + off);
  msg.y = bytestofloat(buf, 1 * 4 + off);
  msg.z = bytestofloat(buf, 2 * 4 + off);
  msg.qw = bytestofloat(buf, 3 * 4 + off);
  msg.qx = bytestofloat(buf, 4 * 4 + off);
  msg.qy = bytestofloat(buf, 5 * 4 + off);
  msg.qz = bytestofloat(buf, 6 * 4 + off);
  msg.vx = bytestofloat(buf, 7 * 4 + off);
  msg.vy = bytestofloat(buf, 8 * 4 + off);
  msg.vz = bytestofloat(buf, 9 * 4 + off);
  msg.wx = bytestofloat(buf, 10 * 4 + off);
  msg.wy = bytestofloat(buf, 11 * 4 + off);
  msg.wz = bytestofloat(buf, 12 * 4 + off);
  return msgid == StateMsg::MsgID;
}

void StateMsgToBytes(const StateMsg& msg, uint8_t* buf, int off) {
  buf[off] = StateMsg::MsgID;
  ++off;
  floattobytes(buf, msg.x, 0 * 4 + off);
  floattobytes(buf, msg.y, 1 * 4 + off);
  floattobytes(buf, msg.z, 2 * 4 + off);
  floattobytes(buf, msg.qw, 3 * 4 + off);
  floattobytes(buf, msg.qx, 4 * 4 + off);
  floattobytes(buf, msg.qy, 5 * 4 + off);
  floattobytes(buf, msg.qz, 6 * 4 + off);
  floattobytes(buf, msg.vx, 7 * 4 + off);
  floattobytes(buf, msg.vy, 8 * 4 + off);
  floattobytes(buf, msg.vz, 9 * 4 + off);
  floattobytes(buf, msg.wx, 10 * 4 + off);
  floattobytes(buf, msg.wy, 11 * 4 + off);
  floattobytes(buf, msg.wz, 12 * 4 + off);
}

bool IMUMeasurementMsgFromBytes(IMUMeasurementMsg& msg, uint8_t* buf, int off) {
  const uint8_t msgid = buf[off];
  ++off;
  msg.ax = bytestofloat(buf, 0 * 4 + off);
  msg.ay = bytestofloat(buf, 1 * 4 + off);
  msg.az = bytestofloat(buf, 2 * 4 + off);
  msg.wx = bytestofloat(buf, 3 * 4 + off);
  msg.wy = bytestofloat(buf, 4 * 4 + off);
  msg.wz = bytestofloat(buf, 5 * 4 + off);
  return msgid == IMUMeasurementMsg::MsgID;
}

void IMUMeasurementMsgToBytes(const IMUMeasurementMsg& msg, uint8_t* buf, int off) {
  buf[off] = IMUMeasurementMsg::MsgID;
  ++off;
  floattobytes(buf, msg.ax, 0 * 4 + off);
  floattobytes(buf, msg.ay, 1 * 4 + off);
  floattobytes(buf, msg.az, 2 * 4 + off);
  floattobytes(buf, msg.wx, 3 * 4 + off);
  floattobytes(buf, msg.wy, 4 * 4 + off);
  floattobytes(buf, msg.wz, 5 * 4 + off);
}

}  // namespace rexquad