extern "C" {
#include "common/delayed_mekf.h"
#include <slap/slap.h>
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
  double x0[13] = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double b0[6];
  double Pf0[225];
  rexquad_SetIdentity(m, Wf, 1e-4);
  rexquad_SetIdentity(e, Vf, 1e-6);
  (void)x0;
  rexquad_SetConstant(m, 1, b0, 0.1);
  rexquad_SetIdentity(e, Pf0, 1.0);
  rexquad_InitializeDelayedMEKF(&filter, delay_comp, Wf, Vf, x0, b0, Pf0);
  double xf[16] = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  const double* xf_ = rexquad_GetFilterState(&filter);
  double err = rexquad_NormSquaredDifference(n, xf, xf_);
  EXPECT_LT(err, 1e-10);
  rexquad_FreeDelayedMEKF(&filter);
}

TEST(DelayedMEKFTests, StatePrediction) {
  int delay_comp = 10;
  const int n = 16;
  const int e = 15;
  const int m = 6;
  // const int n0 = 13;

  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
  rexquad_InitializeDelayMEKFDefault(&filter);

  double xd_[16] = {1.0, 0.2, 1.3, 0.999, 0.0, 0.0, 0.04362, 0.1, -0.2, 0.3, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3};
  double xp_expected_[16] = {1.0011704038156, 0.19809495616880002, 1.3029997111132001, 0.9989978015175386, 0.00014548799745396022, 0.00010644299813724764, 0.04366994923577589, 0.1004395926454363, -0.19954937215069798, 0.20228941296344716, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3};
  double y_imu_[6] = {0.25, 0.24, 0.23, 0.33, 0.32, 0.31};
  Matrix Pd = slap_NewMatrixZeros(e, e);
  Matrix xd = slap_MatrixFromArray(n, 1, xd_);
  Matrix y_imu = slap_MatrixFromArray(m, 1, y_imu_);
  Matrix xp_expected = slap_MatrixFromArray(n, 1, xp_expected_);
  (void)xp_expected;
  double h = 0.01;
  rexquad_StatePrediction(&filter, xd.data, y_imu.data, Pd.data, h);
  Matrix xp = slap_MatrixFromArray(16, 1, filter.xp);
  double err = slap_MatrixNormedDifference(&xp, &xp_expected);
  EXPECT_LT(err, 1e-10);

  rexquad_FreeDelayedMEKF(&filter);
  slap_FreeMatrix(&Pd);
}

}  // namespace rexquad