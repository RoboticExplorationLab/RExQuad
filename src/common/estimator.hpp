#pragma once

#include <deque>
#include <utility>
#include <vector>

#include "constants.hpp"
#include "messages.hpp"
#include "pose.hpp"

namespace rexquad {

struct LinearAcceleration {
  float x;
  float y;
  float z;
  uint64_t timestamp;
};

class StateEstimator {
 public:
  void IMUMeasurement(const IMUMeasurementMsg& imu, uint64_t timestamp);
  void PoseMeasurement(const PoseMsg& pose, uint64_t timestamp_us);
  void GetStateEstimate(float* xhat) const;
  void GetStateEstimate(StateControlMsg& xhat) const;
  void GetStateEstimate(StateVector& xhat) const;
  void SetBias(float* bias);
  void SetIntegrateLinearAccel(bool flag);
  void SetPoseLatency(float latency_s);
  void SetPoseTimestep(float timestep_s);

 private:
  static double ElapsedTime(uint64_t t1_us, uint64_t t2_us);
  float h_pos_;            // timestep for position measurements (seconds)
  float latency_pos_;      // latency for position measurements (seconds)
  uint64_t tprev_pos_us_;  // previous pose timestep
  float xhat_[13];         // current state estimate
  float posprev_[3];       // previous position from MOCAP
  float bias_[6];          // bias on the accelerometer
  std::deque<LinearAcceleration> ahist_;  // Linear acceleration history

  bool do_integrate_linear_accel_ = false;
};

}  // namespace rexquad