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