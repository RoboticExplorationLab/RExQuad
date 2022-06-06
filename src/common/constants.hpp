#pragma once

#include "ArduinoEigenDense.h"
#include "osqp/types.h"
#include "mpc_types.hpp"

namespace rexquad {

using mpc_float = c_float;

constexpr int kNumStates = 13;
constexpr int kNumErrStates = 12;
constexpr int kNumInputs = 4;
// constexpr double kHoverInput = 1427.3354062457752;
constexpr double kHoverInput = 1449.8408445684368;
constexpr double kMinInput = 1148.0;
constexpr double kIdleInput = 1180;
constexpr double kMaxInput = 1832.0;

using StateVector = Eigen::Vector<double, kNumStates>;
using ErrorVector = Eigen::Vector<double, kNumErrStates>;
using InputVector = Eigen::Vector<double, kNumInputs>;
using StateMatrix = Eigen::Matrix<double, kNumErrStates, kNumErrStates>;
using InputMatrix = Eigen::Matrix<double, kNumInputs, kNumInputs>;
using FeedbackGain = Eigen::Matrix<double, kNumInputs, kNumErrStates>;

}  // namespace rexquad