extern "C" {
  #include "common/delayed_mekf.h"
  #include "common/linear_algebra.h"
}

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

namespace rexquad {

  TEST(DelayedMEKFTests, ConstructAndFree) {
    rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(10);
    rexquad_FreeDelayedMEKF(&filter);
  }

  TEST(DelayedMEKFTests, Initialize) {
    int delay_comp = 10;
    const int n = 16;
    const int e = 15;
    const int m = 6;
    // const int n0 = 13;

    rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
    double Wf[36];
    double Vf[225];
    double x0[13] = {0,0,0, 1,0,0,0, 0,0,0, 0,0,0};
    double b0[6];
    double Pf0[225];
    rexquad_SetIdentity(m, Wf, 1e-4);
    rexquad_SetIdentity(e, Vf, 1e-6);
    (void)x0;
    rexquad_SetConstant(m, 1, b0, 0.1);
    rexquad_SetIdentity(e, Pf0, 1.0);
    rexquad_InitializeDelayedMEKF(&filter, delay_comp, Wf, Vf, x0, b0, Pf0);
    double xf[16] = {0,0,0, 1,0,0,0, 0,0,0, 0.1,0.1,0.1, 0.1,0.1,0.1};
    const double* xf_ = rexquad_GetFilterState(&filter);
    double err = rexquad_NormSquaredDifference(n, xf, xf_);
    EXPECT_LT(err, 1e-10);
    rexquad_FreeDelayedMEKF(&filter);
  }

}  // namespace rexquad