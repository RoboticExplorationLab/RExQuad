#include "delayed_mekf.h"

#include <slap/slap.h>
#include <slap/submatrix.h>
#include <stdio.h>
#include <stdlib.h>

#include "linear_algebra.h"
#include "rotations.h"

rexquad_DelayedMEKF rexquad_NewDelayedMEKF(int delay_comp) {
  const int m = 6;
  const int n = 16;  // dimension of filter state
  const int e = 15;  // dimension of error filter state
  const int n0 = 13;
  int total_doubles = m + e + 4 * n + 4 * e * e + n0 + REXQUAD_QUEUE_SIZE * m;

  // Allocate all of the data needed in one block
  double* data = (double*)malloc(total_doubles * sizeof(double));

  // Assign the memory for each field
  double* Wf = data;
  double* Vf = Wf + m;
  double* xf = Vf + e;
  double* Pf = xf + n;
  double* xd = Pf + e * e;
  double* Pd = xd + n;
  double* xp = Pd + e * e;
  double* Pp = xp + n;
  double* xn = Pp + e * e;
  double* Pn = xn + n;
  double* xhat = Pn + e * e;
  double* imuhist0 = xhat + n;

  // Assign the data for the history of IMU measurements
  rexquad_VectorQueue imuhist = rexquad_VectorQueueCreate(imuhist0, m);

  // Create filter
  rexquad_DelayedMEKF filter = {
      .delay_comp = delay_comp,
      .Wf = Wf,
      .Vf = Vf,
      .xf = xf,
      .Pf = Pf,
      .xd = xd,
      .Pd = Pd,
      .xp = xp,
      .Pp = Pp,
      .xn = xn,
      .Pn = Pn,
      .xhat = xhat,
      .imuhist = imuhist,
  };

  return filter;
}

void rexquad_FreeDelayedMEKF(rexquad_DelayedMEKF* filter) { free(filter->Wf); }

void rexquad_InitializeDelayedMEKF(rexquad_DelayedMEKF* filter, int delay_comp,
                                   const double* x0, const double* Wf, const double* Vf,
                                   const double* b0, const double* Pf0) {
  const int m = 6;
  const int n = 16;
  const int e = n - 1;
  const int n0 = 13;

  filter->delay_comp = delay_comp;

  // Copy initial state
  for (int i = 0; i < n0; ++i) {
    filter->xhat[i] = x0[i];
  }

  // Copy position, attitude, and linear velocity from state to filter state
  for (int i = 0; i < 10; ++i) {
    filter->xf[i] = x0[i];
  }

  // (optional) Copy initial bias values into filter state
  for (int i = 0; i < m; ++i) {
    double val = b0 ? b0[i] : 0.0;
    filter->xf[i + 10] = val;
  }

  // (optional) Copy noise covariances
  for (int i = 0; i < m; ++i) {
    double val = Wf ? Wf[i] : 1e-4;
    filter->Wf[i] =  val;
  }
  if (Vf) {
    for (int i = 0; i < e; ++i) {
      filter->Vf[i] = Vf[i];
    }
  } else {
    for (int i = 0; i < 9; ++i) {
      filter->Vf[i] = 1e-4;
    }
    for (int i = 0; i < 6; ++i) {
      filter->Vf[i+9] = 1e-6;
    }
  }

  // (optional) Copy initial covariance
  if (Pf0) {
    for (int i = 0; i < e * e; ++i) {
      filter->Pf[i] = Pf0[i];
      filter->Pd[i] = Pf0[i];
      filter->Pn[i] = Pf0[i];
    }
  } else {
    Matrix Pf = slap_MatrixFromArray(e, e, filter->Pf);
    Matrix Pd = slap_MatrixFromArray(e, e, filter->Pd);
    Matrix Pp = slap_MatrixFromArray(e, e, filter->Pn);
    Matrix Pn = slap_MatrixFromArray(e, e, filter->Pn);
    slap_MatrixSetIdentity(&Pf, 1.0);
    slap_MatrixSetIdentity(&Pd, 1.0);
    slap_MatrixSetIdentity(&Pp, 1.0);
    slap_MatrixSetIdentity(&Pn, 1.0);
  }

  // Copy filter state to other states
  for (int i = 0; i < n; ++i) {
    filter->xd[i] = filter->xf[i];
    filter->xp[i] = filter->xf[i];
    filter->xn[i] = filter->xf[i];
  }
}

void rexquad_InitializeDelayMEKFDefault(rexquad_DelayedMEKF* filter) {
  Matrix Wf = slap_MatrixFromArray(6, 1, filter->Wf);
  Matrix Vf = slap_MatrixFromArray(15, 1, filter->Vf);
  Matrix xf = slap_MatrixFromArray(16, 1, filter->xf);
  Matrix Pf = slap_MatrixFromArray(15, 15, filter->Pf);
  Matrix xd = slap_MatrixFromArray(16, 1, filter->xd);
  Matrix Pd = slap_MatrixFromArray(15, 15, filter->Pd);
  Matrix xhat = slap_MatrixFromArray(13, 1, filter->xhat);
  slap_MatrixSetConst(&Wf, 0.0001);
  slap_MatrixSetConst(&Vf, 0.0001);
  for (int i = 0; i < 6; ++i) {
    slap_MatrixSetElement(&Vf, i + 9, 0, 1e-6);
  }
  slap_MatrixSetIdentity(&Pf, 1.0);
  slap_MatrixSetIdentity(&Pd, 1.0);
  slap_MatrixSetConst(&xf, 0.0);
  slap_MatrixSetElement(&xf, 3, 1, 1.0);
  slap_MatrixCopy(&xd, &xf);
  slap_MatrixSetConst(&xhat, 0.0);
  slap_MatrixSetElement(&xhat, 3, 1, 1.0);
}

const double* rexquad_GetFilterState(const rexquad_DelayedMEKF* filter) {
  return filter->xf;
}

const double* rexquad_GetStateEstimate(const rexquad_DelayedMEKF* filter) {
  return filter->xhat;
}
inline const double* rexquad_GetPredictedState(const rexquad_DelayedMEKF* filter) {
  return filter->xp;
}
inline const double* rexquad_GetPredictedCovariance(const rexquad_DelayedMEKF* filter) {
  return filter->Pp;
}
inline const double* rexquad_GetUpdatedState(const rexquad_DelayedMEKF* filter) {
  return filter->xn;
}
inline const double* rexquad_GetUpdatedCovariance(const rexquad_DelayedMEKF* filter) {
  return filter->Pn;
}

void rexquad_StatePrediction(rexquad_DelayedMEKF* filter, const double* xf,
                             const double* uf, const double* Pf, double h) {
  const Matrix rf = slap_MatrixFromArray(3, 1, (double*)xf + 0);
  const Matrix qf = slap_MatrixFromArray(4, 1, (double*)xf + 3);
  const Matrix vf = slap_MatrixFromArray(3, 1, (double*)xf + 7);
  const Matrix ab = slap_MatrixFromArray(3, 1, (double*)xf + 10);
  const Matrix wb = slap_MatrixFromArray(3, 1, (double*)xf + 13);
  const Matrix af = slap_MatrixFromArray(3, 1, (double*)uf + 0);
  const Matrix wf = slap_MatrixFromArray(3, 1, (double*)uf + 3);

  // Predicted state
  Matrix rp = slap_MatrixFromArray(3, 1, filter->xp + 0);
  Matrix qp = slap_MatrixFromArray(4, 1, filter->xp + 3);
  Matrix vp = slap_MatrixFromArray(3, 1, filter->xp + 7);
  Matrix ap = slap_MatrixFromArray(3, 1, filter->xp + 10);
  Matrix wp = slap_MatrixFromArray(3, 1, filter->xp + 13);
  Matrix Pp = slap_MatrixFromArray(15, 15, filter->Pp);

  // Calculate predicted (corrected) IMU terms
  //   ahat = af - ab
  //   what = wf - wb
  double ahat_[3];
  double what_[3];
  Matrix ahat = slap_MatrixFromArray(3, 1, ahat_);
  Matrix what = slap_MatrixFromArray(3, 1, what_);
  slap_MatrixAddition(&ahat, &af, &ab, -1.0);
  slap_MatrixAddition(&what, &wf, &wb, -1.0);

  // Calculate useful quaternion matrices
  double Qf_[9];
  double g_[3] = {0, 0, 9.81};
  double L_[16];
  double R_[16];
  double G_[12];
  double qf_inv_[3];
  Matrix Qf = slap_MatrixFromArray(3, 3, Qf_);
  Matrix g = slap_MatrixFromArray(3, 1, g_);
  Matrix L = slap_MatrixFromArray(4, 4, L_);
  Matrix R = slap_MatrixFromArray(4, 4, R_);
  Matrix G = slap_MatrixFromArray(4, 3, G_);
  Matrix qf_inv = slap_MatrixFromArray(4, 1, qf_inv_);

  qmat_quat2rotmat(Qf.data, qf.data);
  qmat_lmat(L.data, qf.data);
  qmat_rmat(R.data, qf.data);
  qmat_gmat(G.data, qf.data);
  qmat_inv(qf_inv.data, qf.data);

  // Calculate
  //  phi1 = -0.5 * h * (wf - wb)
  //  phi2 = +0.5 * h * (wf - wb)
  double phi1_[3];
  double phi2_[3];
  Matrix phi1 = slap_MatrixFromArray(3, 1, phi1_);
  Matrix phi2 = slap_MatrixFromArray(3, 1, phi2_);
  slap_MatrixCopy(&phi1, &what);
  slap_MatrixCopy(&phi2, &what);
  slap_MatrixScale(&phi1, -0.5 * h);
  slap_MatrixScale(&phi2, +0.5 * h);

  // Calculate
  //   y1 = cay(phi1)
  //   y2 = cay(phi2)
  double y1_[4];
  double y2_[4];
  double Y1_[9];
  Matrix y1 = slap_MatrixFromArray(4, 1, y1_);
  Matrix y2 = slap_MatrixFromArray(4, 1, y2_);
  Matrix Y1 = slap_MatrixFromArray(3, 3, Y1_);
  qmat_cay(y1.data, phi1.data);
  qmat_cay(y2.data, phi2.data);
  qmat_quat2rotmat(Y1.data, y1.data);

  // Predicted position
  //  rp = rf + Qf * vf
  double vi_[3];
  Matrix vi = slap_MatrixFromArray(3, 1, vi_);  // velocity in inertial (world) frame
  qmat_rotate(vi.data, qf.data, vf.data);
  slap_MatrixAddition(&rp, &rf, &vi, h);

  // Predicted quaternion
  //  qp = L(qf) * cay(phi2)
  slap_MatrixMultiply(&qp, &L, &y2, 0, 0, 1.0, 0.0);

  // Velocity in the old body frame
  //  vpk = vf + h * (ahat - Qf'g)
  double gi_[3];
  double vpk_[3];
  Matrix vpk = slap_MatrixFromArray(3, 1, vpk_);
  Matrix gi = slap_MatrixFromArray(3, 1, gi_);  // gravity in inertial (world) frame
  qmat_rotate(gi.data, qf.data, g.data);
  slap_MatrixCopy(&vpk, &ahat);
  slap_MatrixAddition(&vpk, &vpk, &gi, -1.0);
  slap_MatrixAddition(&vpk, &vf, &vpk, h);

  // Velocity in the new body frame
  //  vp = Y * vpk
  slap_MatrixMultiply(&vp, &Y1, &vpk, 0, 0, 1.0, 0.0);

  // Predicted biases (keep the same)
  slap_MatrixCopy(&ap, &ab);
  slap_MatrixCopy(&wp, &wb);

  // Derivative of rp wrt q
  //  dvdp = h * drotate(qf, vf) * G(qf)
  double drot_[12];
  double drdp_[9];
  Matrix drdq = slap_MatrixFromArray(3, 4, drot_);
  Matrix drdp = slap_MatrixFromArray(3, 3, drdp_);
  qmat_drotate(drdq.data, qf.data, vf.data);
  slap_MatrixMultiply(&drdp, &drdq, &G, 0, 0, h, 0.0);

  // Derivative of vp wrt q (rotation error)
  //  dvdp = -h * Y * drotate(qf' g) * G(qf)
  double dgdp_[9];
  double dvdp_[9];
  Matrix dgdq = slap_MatrixFromArray(3, 4, drot_);
  Matrix dgdp = slap_MatrixFromArray(3, 3, dgdp_);
  Matrix dvdp = slap_MatrixFromArray(3, 3, dvdp_);
  qmat_drotate(dgdq.data, qf_inv.data, g.data);
  slap_MatrixMultiply(&dgdp, &dgdq, &G, 0, 0, 1.0, 0.0);
  slap_MatrixMultiply(&dvdp, &Y1, &dgdp, 0, 0, -h, 0.0);

  // Derivative of vp wrt wb
  //  dvdb = 0.5 * h * drotate(y, vpk) * dcay(phi1)
  double dcay_[12];
  double dvdb_[9];
  Matrix dcay = slap_MatrixFromArray(4, 3, dcay_);
  Matrix dydq = slap_MatrixFromArray(3, 4, drot_);
  Matrix dvdb = slap_MatrixFromArray(3, 3, dvdb_);
  qmat_drotate(dydq.data, y1.data, vpk.data);
  qmat_dcay(dcay.data, phi1.data);
  slap_MatrixMultiply(&dvdb, &dydq, &dcay, 0, 0, 0.5 * h, 0.0);

  // Derivative of qp wrt wb
  //  dqdb = -0.5 * h * G(qp)'L(qf) * dcay(phi2)
  double dqdb_[9];
  Matrix GtL = slap_MatrixFromArray(3, 4, drot_);  // steal data from drot
  Matrix dqdb = slap_MatrixFromArray(3, 3, dqdb_);
  qmat_gmat(G.data, qp.data);
  qmat_dcay(dcay.data, phi2.data);
  slap_MatrixMultiply(&GtL, &G, &L, 1, 0, 1.0, 0.0);
  slap_MatrixMultiply(&dqdb, &GtL, &dcay, 0, 0, -0.5 * h, 0.0);

  // Calculate Jacobian
  double Af_[225];
  Matrix Af = slap_MatrixFromArray(15, 15, Af_);
  slap_MatrixSetConst(&Af, 0.0);
  SubMatrix Af_drdr = slap_SubMatrixFromMatrix(0, 0, 3, 3, &Af);
  slap_SubMatrixSetIdentity(&Af_drdr, 1.0);
  SubMatrix Af_drdp = slap_SubMatrixFromMatrix(0, 3, 3, 3, &Af);
  slap_SubMatrixCopyFromMatrix(&Af_drdp, &drdp);
  SubMatrix Af_drdv = slap_SubMatrixFromMatrix(0, 6, 3, 3, &Af);
  slap_SubMatrixCopyWithScaling(&Af_drdv, &Qf, h);
  SubMatrix Af_dpdp = slap_SubMatrixFromMatrix(3, 3, 3, 3, &Af);
  slap_SubMatrixCopyFromMatrix(&Af_dpdp, &Y1);
  SubMatrix Af_dpdw = slap_SubMatrixFromMatrix(3, 12, 3, 3, &Af);
  slap_SubMatrixCopyFromMatrix(&Af_dpdw, &dqdb);
  SubMatrix Af_dvdp = slap_SubMatrixFromMatrix(6, 3, 3, 3, &Af);
  slap_SubMatrixCopyFromMatrix(&Af_dvdp, &dvdp);
  SubMatrix Af_dvdv = slap_SubMatrixFromMatrix(6, 6, 3, 3, &Af);
  slap_SubMatrixCopyFromMatrix(&Af_dvdv, &Y1);
  SubMatrix Af_dvda = slap_SubMatrixFromMatrix(6, 9, 3, 3, &Af);
  slap_SubMatrixCopyWithScaling(&Af_dvda, &Y1, -h);
  SubMatrix Af_dvdw = slap_SubMatrixFromMatrix(6, 12, 3, 3, &Af);
  slap_SubMatrixCopyFromMatrix(&Af_dvdw, &dvdb);
  SubMatrix Af_dbdb = slap_SubMatrixFromMatrix(9, 9, 6, 6, &Af);
  slap_SubMatrixSetIdentity(&Af_dbdb, 1.0);

  // Calculate Predicted Covariance
  double PAt_[225];
  Matrix PAt = slap_MatrixFromArray(15, 15, PAt_);
  slap_MatrixSetConst(&PAt, 0.0);
  const Matrix Pf_mat = slap_MatrixFromArray(15, 15, (double*)Pf);
  // slap_MatrixCopyFromArray(&Pf_mat, Pf);
  slap_MatrixSetConst(&Pp, 0.0);
  slap_MatrixSetDiagonal(&Pp, filter->Vf);

  slap_MatrixMultiply(&PAt, &Pf_mat, &Af, 0, 1, 1.0, 0.0);
  slap_MatrixMultiply(&Pp, &Af, &PAt, 0, 0, 1.0, 1.0);

  (void)Pf;
  (void)Pp;
}
void rexquad_MeasurementUpdate(rexquad_DelayedMEKF* filter, const double* xf,
                               const double* Pf, const double* y_mocap) {
  // Split up the filter state
  const Matrix rf = slap_MatrixFromArray(3, 1, (double*)xf + 0);
  const Matrix qf = slap_MatrixFromArray(4, 1, (double*)xf + 3);
  const Matrix vf = slap_MatrixFromArray(3, 1, (double*)xf + 7);
  const Matrix ab = slap_MatrixFromArray(3, 1, (double*)xf + 10);
  const Matrix wb = slap_MatrixFromArray(3, 1, (double*)xf + 13);

  // Split up the MOCAP measurement
  const Matrix rm = slap_MatrixFromArray(3, 1, (double*)y_mocap + 0);
  const Matrix qm = slap_MatrixFromArray(4, 1, (double*)y_mocap + 3);

  // Split up the updated state
  Matrix rn = slap_MatrixFromArray(3, 1, filter->xn + 0);
  Matrix qn = slap_MatrixFromArray(4, 1, filter->xn + 3);
  Matrix vn = slap_MatrixFromArray(3, 1, filter->xn + 7);
  Matrix an = slap_MatrixFromArray(3, 1, filter->xn + 10);
  Matrix wn = slap_MatrixFromArray(3, 1, filter->xn + 13);

  const Matrix Pf_mat = slap_MatrixFromArray(15, 15, (double*)Pf);
  Matrix Pn = slap_MatrixFromArray(15, 15, filter->Pn);

  // Innovation
  double z_[6];
  Matrix z = slap_MatrixFromArray(6, 1, z_);
  Matrix dr = slap_MatrixFromArray(3, 1, z_ + 0);
  Matrix dq = slap_MatrixFromArray(4, 1, z_ + 3);
  slap_MatrixAddition(&dr, &rm, &rf, -1.0);
  qmat_err(dq.data, qm.data, qf.data);

  // Measurement Jacobian
  double Cf_[90];
  Matrix Cf = slap_MatrixFromArray(6, 15, Cf_);
  slap_MatrixSetIdentity(&Cf, 1.0);

  // Kalman gain
  //   S = Cf * Pf * Cf' + Wf
  double CPt_[90];
  double S_[36];
  Matrix CP = slap_MatrixFromArray(6, 15, CPt_);
  Matrix S = slap_MatrixFromArray(6, 6, S_);
  slap_MatrixSetConst(&CP, 0.0);
  slap_MatrixSetConst(&S, 0.0);
  slap_MatrixSetDiagonal(&S, filter->Wf);
  slap_MatrixMultiply(&CP, &Cf, &Pf_mat, 0, 0, 1.0, 0.0);
  slap_MatrixMultiply(&S, &CP, &Cf, 0, 1, 1.0, 1.0);

  //   Lt = S\(Cf*Pf)
  Matrix* Lt = &CP;  // Kalman gain transposed
  slap_CholeskyFactorize(&S);
  slap_CholeskySolve(&S, Lt);

  // Update mean
  double dx_[15];
  Matrix dx = slap_MatrixFromArray(15, 1, dx_);
  slap_MatrixSetConst(&dx, 0.0);
  slap_MatrixMultiply(&dx, Lt, &z, 1, 0, 1.0, 0.0);

  dr = slap_MatrixFromArray(3, 1, dx_ + 0);
  dq = slap_MatrixFromArray(3, 1, dx_ + 3);
  Matrix dv = slap_MatrixFromArray(3, 1, dx_ + 6);
  Matrix da = slap_MatrixFromArray(3, 1, dx_ + 9);
  Matrix dw = slap_MatrixFromArray(3, 1, dx_ + 12);

  slap_MatrixAddition(&rn, &rf, &dr, 1.0);
  qmat_adderr(qn.data, qf.data, dq.data);
  slap_MatrixAddition(&vn, &vf, &dv, 1.0);
  slap_MatrixAddition(&an, &ab, &da, 1.0);
  slap_MatrixAddition(&wn, &wb, &dw, 1.0);

  // Update covariance
  //  Pn = (I-Lf*Cf)*Pf*(I-Lf*Cf)' + Lf*Wf*Lf'
  double LC_[225];
  double tmp_[225];
  Matrix LC = slap_MatrixFromArray(15, 15, LC_);
  Matrix tmp = slap_MatrixFromArray(15, 15, tmp_);
  slap_MatrixSetConst(&LC, 0.0);
  slap_MatrixMultiply(&LC, Lt, &Cf, 1, 0, -1.0, 0.0);
  slap_AddIdentity(&LC, 1.0);
  slap_MatrixMultiply(&tmp, &Pf_mat, &LC, 0, 1, 1.0, 0.0);
  slap_MatrixMultiply(&Pn, &LC, &tmp, 0, 0, 1.0, 0.0);

  tmp = slap_MatrixFromArray(6, 15, tmp_);  // use the memory from the previous temp array
  Matrix Wf_diag = slap_MatrixFromArray(6, 1, filter->Wf);
  slap_DiagonalMultiplyLeft(&tmp, &Wf_diag, Lt);
  slap_MatrixMultiply(&Pn, Lt, &tmp, 1, 0, 1.0, 1.0);

  (void)Pf;
}

void rexquad_CacheIMUMeasurement(rexquad_DelayedMEKF* filter, const double* y_imu) {
  rexquad_VectorQueuePush(&filter->imuhist, y_imu);
}

int rexquad_GetIMUCacheLength(const rexquad_DelayedMEKF* filter) {
  return rexquad_VectorQueueSize(&filter->imuhist);
}

const double* rexquad_GetDelayedIMUMeasurement(const rexquad_DelayedMEKF* filter,
                                               int delay) {
  return rexquad_VectorQueueGet(&filter->imuhist, delay);
}

void rexquad_PopLastIMUMeasurement(rexquad_DelayedMEKF* filter) {
  rexquad_VectorQueuePop(&filter->imuhist);
}

void rexquad_UpdateStateEstimate(rexquad_DelayedMEKF* filter, const double* y_imu,
                                 const double* y_mocap, double h) {
  int n = 16;
  int e = n - 1;

  // Delayed filter state
  puts("Get delayed filter state");
  Matrix xd = slap_MatrixFromArray(n, 1, filter->xd);
  Matrix Pd = slap_MatrixFromArray(e, e, filter->Pd);

  // Updated filter state (after measurement update)
  Matrix xn = slap_MatrixFromArray(n, 1, filter->xn);
  Matrix Pn = slap_MatrixFromArray(e, e, filter->Pn);

  // Current filter state
  Matrix xf = slap_MatrixFromArray(n, 1, filter->xf);
  Matrix Pf = slap_MatrixFromArray(e, e, filter->Pf);

  // Predicted filter state (after prediction step)
  Matrix xp = slap_MatrixFromArray(n, 1, filter->xp);
  Matrix Pp = slap_MatrixFromArray(e, e, filter->Pp);

  // Cache the IMU data
  // NOTE: this method keeps the history at a given length
  puts("Caching IMU Measurement");
  rexquad_CacheIMUMeasurement(filter, y_imu);

  // Get the estimated delay (in number of time steps
  int delay = filter->delay_comp;
  int imu_hist_length = rexquad_GetIMUCacheLength(filter);
  if (delay > imu_hist_length - 1) {
    delay = imu_hist_length - 1;
  }
  printf("Using delay of %d\n", delay);

  // Use MOCAP measurement to update the delayed filter state
  printf("xd0: ");
  slap_PrintRowVector(&xd);
  printf("Pd:\n");
  slap_PrintMatrix(&Pd);
  if (y_mocap != NULL) {
    puts("Processing MOCAP measurement");
    // Get the delayed IMU measurement
    const double* y_imu_delayed = rexquad_GetDelayedIMUMeasurement(filter, delay);

    // Advance the delayed measurement using the past IMU measurement
    //  Updates filter->xp, filter->Pp
    rexquad_StatePrediction(filter, xd.data, y_imu_delayed, Pd.data, h);
    printf("xp: ");
    slap_PrintRowVector(&xp);
    printf("Pp:\n");
    slap_PrintMatrix(&Pp);

    // Update the delayed filtered estimate using the MOCAP measurement
    //   Updates filter->xn, filter->Pn
    rexquad_MeasurementUpdate(filter, xp.data, Pp.data, y_mocap);
    printf("xn: ");
    slap_PrintRowVector(&xn);

    // Copy the updated state to the delayed state
    slap_MatrixCopy(&xd, &xn);
    slap_MatrixCopy(&Pd, &Pn);

    // Pop off the last IMU Measurement since it's been processed
    if (rexquad_VectorQueueSize(&filter->imuhist) > filter->delay_comp) {
      rexquad_PopLastIMUMeasurement(filter);
    }
  }

  // Use history of IMU data to predict the state at the current time
  puts("Predicting the current state using IMU");
  printf("xd: ");
  slap_PrintRowVector(&xd);
  slap_MatrixCopy(&xf, &xd);
  slap_MatrixCopy(&Pf, &Pd);
  for (int i = 0; i < delay - 1; ++i) {
    const double* y_imu_delayed = rexquad_GetDelayedIMUMeasurement(filter, delay - i);

    rexquad_StatePrediction(filter, xf.data, y_imu_delayed, Pd.data, h);
    slap_MatrixCopy(&xf, &xp);
    slap_MatrixCopy(&Pf, &Pp);
  }

  // Create state estimate from filter state
  puts("Creating state estimate from filter state");
  Matrix xf_rqv = slap_MatrixFromArray(10, 1, filter->xf);
  Matrix w_bias = slap_MatrixFromArray(10, 1, filter->xf + 13);

  // Read Gyro measurement
  const Matrix w_imu = slap_MatrixFromArray(3, 1, (double*)y_imu + 3);

  // Get estimate of angular velocity by subtracting bias from raw gyro data
  //  what = w_gyro - w_bias
  double what_[3];
  Matrix what = slap_MatrixFromArray(3, 1, what_);
  slap_MatrixAddition(&what, &w_imu, &w_bias, -1.0);

  // Copy data into state estimate xhat = [r, q, v, w]
  Matrix xhat_rqv = slap_MatrixFromArray(10, 1, filter->xhat);
  Matrix xhat_w = slap_MatrixFromArray(3, 1, filter->xhat + 10);
  slap_MatrixCopy(&xhat_rqv, &xf_rqv);
  slap_MatrixCopy(&xhat_w, &what);
}
