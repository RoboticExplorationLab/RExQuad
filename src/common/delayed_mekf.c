#include "delayed_mekf.h"

#include <stdlib.h>

#include "linear_algebra.h"

rexquad_DelayedMEKF rexquad_NewDelayedMEKF(int delay_comp) {
  const int m = 6;
  const int n = 16;   // dimension of filter state
  const int e = 15;   // dimension of error filter state
  const int n0 = 13;
  int total_doubles = m + e + 2 * n + 2 * e * e + n0 + MAX_IMU_HISTORY * m;

  double* imuhist[MAX_IMU_HISTORY];

  // Allocate all of the data needed in one block
  double* data = (double*)malloc(total_doubles * sizeof(double));

  // Assign the memory for each field
  double* Wf = data;
  double* Vf = Wf + m;
  double* xf = Vf + e;
  double* Pf = xf + n;
  double* xd = Pf + e * e;
  double* Pd = xd + n;
  double* xhat = Pd + e * e;
  double* imuhist0 = xhat + n;
  for (int i = 0; i < MAX_IMU_HISTORY; ++i) {
    imuhist[i] = imuhist0 + i * m;
  }

  // Create filter
  rexquad_DelayedMEKF filter = {
      .delay_comp = delay_comp,
      .Wf = Wf,
      .Vf = Vf,
      .xf = xf,
      .Pf = Pf,
      .xd = xd,
      .Pd = Pd,
      .xhat = xhat,
      .imuhist = imuhist,
  };

  return filter;
}

void rexquad_FreeDelayedMEKF(rexquad_DelayedMEKF* filter) { free(filter->Wf); }

void rexquad_InitializeDelayedMEKF(rexquad_DelayedMEKF* filter, int delay_comp,
                                   const double* Wf, const double* Vf, const double* x0,
                                   const double* b0, const double* Pf0) {
  const int m = 6;
  const int n = 15;
  const int n0 = 13;

  filter->delay_comp = delay_comp;

  // Copy noise covariances
  for (int i = 0; i < m; ++i) {
    filter->Wf[i] = Wf[i];
  }
  for (int i = 0; i < n; ++i) {
    filter->Vf[i] = Vf[i];
  }

  // Copy initial state
  for (int i = 0; i < n0; ++i) {
    filter->xhat[i] = x0[i];
  }

  // Copy position, attitude, and linear velocity from state to filter state
  for (int i = 0; i < 10; ++i) {
    filter->xf[i] = x0[i];
  }

  // Copy initial bias values into filter state
  for (int i = 0; i < m; ++i) {
    filter->xf[i + 10] = b0[i];
  }

  // Copy initial covariance
  for (int i = 0; i < n * n; ++i) {
    filter->Pf[i] = Pf0[i];
    filter->Pd[i] = Pf0[i];
  }

  // Copy filter state to delayed filtered state
  for (int i = 0; i < n; ++i) {
    filter->xd[i] = filter->xf[i];
  }
}

const double* rexquad_GetFilterState(const rexquad_DelayedMEKF* filter) {
  return filter->xf;
}

const double* rexquad_GetStateEstimate(const rexquad_DelayedMEKF* filter) {
  return filter->xhat;
}

