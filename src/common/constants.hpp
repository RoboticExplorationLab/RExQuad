#pragma once

#include "ArduinoEigenDense.h"
#include "osqp/types.h"
#include "mpc_types.hpp"

namespace rexquad {

// using mpc_float = c_float;

constexpr int kNumStates = 13;
constexpr int kNumErrStates = 12;
constexpr int kNumInputs = 4;
// constexpr double kHoverInput = 1427.3354062457752;
constexpr double kHoverInput = 1449.8408445684368;
constexpr double kMinInput = 1148.0;
constexpr double kIdleInput = 1180;
constexpr double kMaxInput = 1832.0;

using StateVector = Eigen::Vector<mpc_float, kNumStates>;
using ErrorVector = Eigen::Vector<mpc_float, kNumErrStates>;
using InputVector = Eigen::Vector<mpc_float, kNumInputs>;
using StateMatrix = Eigen::Matrix<mpc_float, kNumErrStates, kNumErrStates>;
using InputMatrix = Eigen::Matrix<mpc_float, kNumErrStates, kNumInputs>;
using FeedbackGain = Eigen::Matrix<mpc_float, kNumInputs, kNumErrStates>;

using StatePenalty = Eigen::DiagonalMatrix<mpc_float, kNumErrStates>;
using InputPenalty = Eigen::DiagonalMatrix<mpc_float, kNumInputs>;

}  // namespace rexquad