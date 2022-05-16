#pragma once

#include "messages.hpp"
#include "pose.hpp"

namespace rexquad {

class StateEstimator {
public:
  void IMUMeasurement(const IMUMeasurementMsg& imu, double timestamp);
  void PoseMeasurement(const PoseMsg& pose, double timestamp);
  void GetStateEstimate(float* xhat) const;
  void GetStateEstimate(StateControlMsg& xhat) const;
  void SetBias(float* bias);
  void SetIntegrateLinearAccel(bool flag);

private:
  float xhat_[13];       // current state estimate
  float posprev_[3];     // previous position from MOCAP
  double tprev_pos_;     // timestamp of previous MOCAP position
  float aprev_[3];       // previous linear acceleration
  double tprev_accel_;   // timestamp of previous linear acceleration
  float bias_[6];        // bias on the accelerometer

  bool do_integrate_linear_accel_ = false;
};

}  // namespace rexquad