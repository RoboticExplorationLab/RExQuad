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
  int total_doubles = m + e + 3 * n + 3 * e * e + n0 + MAX_IMU_HISTORY * m;

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
  double* xp = Pd + e * e;
  double* Pp = xd + n;
  double* xhat = Pp + e * e;
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
      .xp = xp,
      .Pp = Pp,
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
  Matrix GtL = slap_MatrixFromArray(3, 4, drot_);   // steal data from drot
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
  // slap_MatrixCopyFromArray(&P)
  slap_MatrixSetConst(&Pp, 0.0);
  slap_MatrixSetDiagonal(&Pp, filter->Vf);

  slap_MatrixMultiply(&PAt, &Pf_mat, &Af, 0, 1, 1.0, 0.0);
  slap_MatrixMultiply(&Pp, &Af, &PAt, 0, 0, 1.0, 1.0);

  (void)Pf;
  (void)Pp;
}
inline const double* rexquad_GetPredictedState(const rexquad_DelayedMEKF* filter) {
  return filter->xp;
}
inline const double* rexquad_GetPredictedCovariance(const rexquad_DelayedMEKF* filter) {
  return filter->Pp;
}
