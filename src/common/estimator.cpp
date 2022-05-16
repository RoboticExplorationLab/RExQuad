#include "estimator.hpp"

#include <cstring>
#include "messages.hpp"

namespace rexquad {

void StateEstimator::IMUMeasurement(const IMUMeasurementMsg &imu, double timestamp) {
  float a[3] = { imu.ax - bias_[0], imu.ay - bias_[1], imu.az - bias_[2] };
  float w[3] = { imu.wx - bias_[3], imu.wy - bias_[4], imu.wz - bias_[5] };
  double dt = timestamp - tprev_;
  for (int i = 0; i < 3; ++i) {
    // Integrate linear acceleration to get linear velocity
    xhat_[7+i] += (a[i] + aprev_[i]) * 0.5 * dt; 
    aprev_[i] = a[i];

    // Use current angular velocity 
    xhat_[10+i] = w[i];
  }
  tprev_ = timestamp;
}

void StateEstimator::PoseMeasurement(const PoseMsg &pose, double timestamp) {
  (void) timestamp;

  // Use pose measurement as ground truth
  xhat_[0] = pose.x;
  xhat_[1] = pose.y;
  xhat_[2] = pose.z;

  xhat_[3] = pose.qw;
  xhat_[4] = pose.qx;
  xhat_[5] = pose.qy;
  xhat_[6] = pose.qz;
}

void StateEstimator::GetStateEstimate(float *xhat) const {
  memcpy(xhat, xhat_, sizeof(xhat_));
}

void StateEstimator::GetStateEstimate(StateControlMsg& xhat) const {
  xhat.x = xhat_[0];
  xhat.y = xhat_[1];
  xhat.z = xhat_[2];
  xhat.qw = xhat_[3];
  xhat.qx = xhat_[4];
  xhat.qy = xhat_[5];
  xhat.qz = xhat_[6];
}

void StateEstimator::SetBias(float *bias) {
  memcpy(bias_, bias, sizeof(bias_));
}

}  // namespace rexquad