#pragma once

#include "ArduinoEigenDense.h"

namespace rexquad {

using StateVector = Eigen::Vector<float, 13>;
using ErrorVector = Eigen::Vector<float, 12>;
using InputVector = Eigen::Vector<float, 4>;
using FeedbackGain = Eigen::Matrix<float, 4, 12>;

constexpr int kNumStates = 13;
constexpr int kNumErrStates = 12;
constexpr int kNumInputs = 4;
// constexpr double kHoverInput = 1427.3354062457752;
constexpr double kHoverInput = 1449.8408445684368;
constexpr double kMinInput = 1148.0;
constexpr double kIdleInput = 1180;
constexpr double kMaxInput = 1832.0;

}  // namespace rexquad