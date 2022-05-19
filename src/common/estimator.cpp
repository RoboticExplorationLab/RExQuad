#include "estimator.hpp"

#include <cstring>

#include "messages.hpp"

namespace rexquad {

double StateEstimator::ElapsedTime(uint64_t t1_us, uint64_t t2_us) {
  return static_cast<double>(t1_us - t2_us) * 1e-6;
}

void StateEstimator::IMUMeasurement(const IMUMeasurementMsg& imu, uint64_t timestamp_us) {
  float w[3] = {imu.wx - bias_[3], imu.wy - bias_[4], imu.wz - bias_[5]};

  for (int i = 0; i < 3; ++i) {
    if (do_integrate_linear_accel_) {
      float a[3] = {imu.ax - bias_[0], imu.ay - bias_[1], imu.az - bias_[2]};
      const LinearAcceleration& accelprev = ahist_.back();
      float aprev[3] = {accelprev.x, accelprev.y, accelprev.z};
      double dt = ElapsedTime(timestamp_us, accelprev.timestamp);
      // Integrate linear acceleration to get linear velocity
      xhat_[7 + i] += (a[i] + aprev[i]) * 0.5 * dt;

      // Append to history
      LinearAcceleration accel = {a[0], a[1], a[2], timestamp_us};
      ahist_.emplace_back(std::move(accel));
      if (ahist_.size() > 10) {  // TODO: calculate this size
        ahist_.pop_front();
      }
      // for (int i = 0; i < ahist_.size(); ++i) {
      //   // Remove any history longer than the 2x the latency
      //   double time_passed = ElapsedTime(timestamp_us, ahist_.front().timestamp);
      //   if (time_passed > h_pos_) {
      //     ahist_.pop_front();
      //   }
      // }
    }

    // Use current angular velocity
    xhat_[10 + i] = w[i];
  }
}

void StateEstimator::PoseMeasurement(const PoseMsg& pose, uint64_t timestamp_us) {
  (void)timestamp_us;

  // Use pose measurement as ground truth
  xhat_[0] = pose.x;
  xhat_[1] = pose.y;
  xhat_[2] = pose.z;

  xhat_[3] = pose.qw;
  xhat_[4] = pose.qx;
  xhat_[5] = pose.qy;
  xhat_[6] = pose.qz;

  // Finite Diff the position to get linear velocity
  double dt_pos = (timestamp_us - tprev_pos_us_) * 1e-6;
  for (int i = 0; i < 3; ++i) {
    xhat_[7 + i] = static_cast<float>((xhat_[i] - posprev_[i]) / dt_pos);

    // Cache previous position
    posprev_[i] = xhat_[i];
  }

  // Iterate backwards through linear acceleration hist
  // for (const auto& it = ahist_.crbegin(); it > ahist_.crend(); ++it) {

  // }
  tprev_pos_us_ = timestamp_us;
}

void StateEstimator::GetStateEstimate(float* xhat) const {
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
  xhat.vx = xhat_[7];
  xhat.vy = xhat_[8];
  xhat.vz = xhat_[9];
  xhat.wx = xhat_[10];
  xhat.wy = xhat_[11];
  xhat.wz = xhat_[12];
}

void StateEstimator::GetStateEstimate(StateVector& xhat) const {
  memcpy(xhat.data(), xhat_, sizeof(xhat_));
}

void StateEstimator::SetBias(float* bias) { memcpy(bias_, bias, sizeof(bias_)); }

void StateEstimator::SetIntegrateLinearAccel(bool flag) {
  do_integrate_linear_accel_ = flag;
}

}  // namespace rexquad