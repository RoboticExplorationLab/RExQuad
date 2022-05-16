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

private:
  float xhat_[13];  // current state estimate
  float aprev_[3];  // previous linear acceleration
  double tprev_;    // timestamp of previous linear acceleration
  float bias_[6];   // bias on the accelerometer
};

}  // namespace rexquad