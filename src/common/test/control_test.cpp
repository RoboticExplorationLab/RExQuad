#include "common/control.hpp"

#include <fmt/core.h>
#include <gtest/gtest.h>
#include <cmath>

namespace rexquad {

TEST(COMMON_TESTS, ErrorStateTest) {
  StateVector x0;
  StateVector x;
  ErrorVector e;
  x0.setZero();
  x.setZero();
  e.setZero();
  x0(3) = 1.0;
  x(3) = 1.0;
  EXPECT_FLOAT_EQ(e.norm(), 0.0);
  ErrorState(e, x, x0);
  EXPECT_FLOAT_EQ(e.norm(), 0.0);

  x(0) = 3;
  x(1) = 4;
  x(2) = 0;
  ErrorState(e, x, x0);
  EXPECT_FLOAT_EQ(e.norm(), 5.0);

  x.setZero();
  x(3) = std::sqrt(2) / 2;
  x(4) = std::sqrt(2) / 2;
  ErrorState(e, x, x0);
  EXPECT_FLOAT_EQ(e.norm(), 1.0);

  x(0) = 3;
  x(1) = 4;
  ErrorState(e, x0, x);
  EXPECT_FLOAT_EQ(e(0), -3.0);
  EXPECT_FLOAT_EQ(e(1), -4.0);
  EXPECT_FLOAT_EQ(e(3), -1.0);
}

}  // namespace rexquad