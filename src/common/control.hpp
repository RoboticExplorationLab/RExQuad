#pragma once

#include "constants.hpp"

namespace rexquad {

void ErrorState(ErrorVector& e, const StateVector& x, const StateVector& xeq);

}  // namespace rexquad