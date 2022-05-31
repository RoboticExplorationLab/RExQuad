#pragma once

#include <inttypes.h>

namespace rexquad {


struct MeasurementMsg {
  static constexpr uint8_t MsgID = 109;  // 'm'
  float x;
  float y;
  float z;
  float qw;
  float qx;
  float qy;
  float qz;
  float vx;
  float vy;
  float vz;
  float ax;
  float ay;
  float az;
  float wx;
  float wy;
  float wz;
};

bool MeasurementMsgFromBytes(MeasurementMsg& msg, uint8_t* buf, int off = 0);
void MeasurementMsgToBytes(const MeasurementMsg& msg, uint8_t* buf, int off = 0);

struct ControlMsg {
  static constexpr uint8_t MsgID = 99;  // 'c'
  float data[4];
};
bool ControlMsgFromBytes(ControlMsg& msg, uint8_t* buf, int off = 0);
void ControlMsgToBytes(const ControlMsg& msg, uint8_t* buf, int off = 0);

template <class T>
void ControlMsgFromVector(ControlMsg& msg, const T* u) {
  for (int i = 0; i < 4; ++i)  {
    msg.data[i] = u[i];
  }
}

/**
 * @brief 
 * 
 * This message is used in simulation mode to send the state estimate and control
 * command from the onboard computer back to the simulator.
 */
struct StateControlMsg {
  static constexpr uint8_t MsgID = 214;  // 's' + 'c'
  float x;
  float y;
  float z;
  float qw;
  float qx;
  float qy;
  float qz;
  float vx;
  float vy;
  float vz;
  float wx;
  float wy;
  float wz;
  float u[4];
};

bool StateControlMsgFromBytes(StateControlMsg& msg, uint8_t* buf, int off = 0);
void StateControlMsgToBytes(const StateControlMsg& msg, uint8_t* buf, int off = 0);

template<class T>
void StateControlMsgFromVectors(StateControlMsg& msg, const T* x, const T* u) {
  msg.x = x[0];
  msg.y = x[1];
  msg.z = x[2];
  msg.qw = x[3];
  msg.qx = x[4];
  msg.qy = x[5];
  msg.qz = x[6];
  msg.vx = x[7];
  msg.vy = x[8];
  msg.vz = x[9];
  msg.wx = x[10];
  msg.wy = x[11];
  msg.wz = x[12];
  for (int i = 0; i < 4; ++i) {
    msg.u[i] = u[i];
  }
}

struct StateMsg {
  static constexpr uint8_t MsgID = 115;  // 's'
  float x;
  float y;
  float z;
  float qw;
  float qx;
  float qy;
  float qz;
  float vx;
  float vy;
  float vz;
  float wx;
  float wy;
  float wz;
};

bool StateMsgFromBytes(StateMsg& msg, uint8_t* buf, int off = 0);
void StateMsgToBytes(const StateMsg& msg, uint8_t* buf, int off = 0);

template<class T>
void StateMsgFromVector(StateMsg& msg, const T* x) {
  msg.x = x[0];
  msg.y = x[1];
  msg.z = x[2];
  msg.qw = x[3];
  msg.qx = x[4];
  msg.qy = x[5];
  msg.qz = x[6];
  msg.vx = x[7];
  msg.vy = x[8];
  msg.vz = x[9];
  msg.wx = x[10];
  msg.wy = x[11];
  msg.wz = x[12];
}

struct IMUMeasurementMsg {
  static constexpr uint8_t MsgID = 105;  // 'i'
  float ax;
  float ay;
  float az;
  float wx;
  float wy;
  float wz;
};
bool IMUMeasurementMsgFromBytes(IMUMeasurementMsg& msg, uint8_t* buf, int off = 0);
void IMUMeasurementMsgToBytes(const IMUMeasurementMsg& msg, uint8_t* buf, int off = 0);


}  // rexquad