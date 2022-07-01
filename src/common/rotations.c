#include "rotations.h"
#include "math.h"

void qmat_inv(double* qinv, const double* q) {
  qinv[0] = q[0];
  qinv[1] = -q[1];
  qinv[2] = -q[2];
  qinv[3] = -q[3];
}

void qmat_quat2rotmat(double* Q, const double* q) {
  Q[0+0] = (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  Q[0+1] = 2 * (q[1] * q[2] + q[0]*q[3]);
  Q[0+2] = 2 * (q[1] * q[3] - q[0]*q[2]);

  Q[3+0] = 2 * (q[1] * q[2] - q[0]*q[3]);
  Q[3+1] = (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]);
  Q[3+2] = 2 * (q[0] * q[1] + q[2]*q[3]);

  Q[6+0] = 2 * (q[0] * q[2] + q[1]*q[3]);
  Q[6+1] = 2 * (q[2] * q[3] - q[0]*q[1]);
  Q[6+2] = (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
}

void qmat_skewmat(double* S, const double* x) {
  S[0] =   0.0;  
  S[1] = +x[2];  
  S[2] = -x[1];  

  S[3] = -x[2];
  S[4] =   0.0;
  S[5] = +x[0];

  S[6] = +x[1];
  S[7] = -x[0];
  S[8] =   0.0;
}

void qmat_lmat(double* L, const double* q) {
  L[0] = q[0];
  L[1] = q[1];
  L[2] = q[2];
  L[3] = q[3];

  L[4] = -q[1];
  L[5] =  q[0];
  L[6] =  q[3];
  L[7] = -q[2];

  L[ 8] = -q[2];
  L[ 9] = -q[3];
  L[10] =  q[0];
  L[11] =  q[1];

  L[12] = -q[3];
  L[13] =  q[2];
  L[14] = -q[1];
  L[15] =  q[0];
}

void qmat_rmat(double* R, const double* q) {
  R[0] = q[0];
  R[1] = q[1];
  R[2] = q[2];
  R[3] = q[3];

  R[4] = -q[1];
  R[5] =  q[0];
  R[6] = -q[3];
  R[7] =  q[2];

  R[ 8] = -q[2];
  R[ 9] =  q[3];
  R[10] =  q[0];
  R[11] = -q[1];

  R[12] = -q[3];
  R[13] = -q[2];
  R[14] =  q[1];
  R[15] =  q[0];
}

void qmat_gmat(double* G, double* q) {
  G[0] = -q[1];
  G[1] =  q[0];
  G[2] =  q[3];
  G[3] = -q[2];

  G[4] = -q[2];
  G[5] = -q[3];
  G[6] =  q[0];
  G[7] =  q[1];

  G[ 8] = -q[3];
  G[ 9] =  q[2];
  G[10] = -q[1];
  G[11] =  q[0];
}

void qmat_cay(double* q, const double* phi) {
  double m = 1.0 / sqrt(1 + phi[0]*phi[0] + phi[1]*phi[1] + phi[2]*phi[2]);
  q[0] = m;
  q[1] = phi[0] * m;
  q[2] = phi[1] * m;
  q[3] = phi[2] * m;
}

void qmat_icay(double* phi, const double* q) {
  phi[0] = q[1] / q[0];
  phi[1] = q[2] / q[0];
  phi[2] = q[3] / q[0];
}

void qmat_dcay(double* D, const double* phi) {
  double m = 1.0 / sqrt(1 + phi[0]*phi[0] + phi[1]*phi[1] + phi[2]*phi[2]);
  double m3 = - m * m * m;
  D[0] = m3 * phi[0];
  D[1] = m3 * phi[0] * phi[0];
  D[2] = m3 * phi[1] * phi[0];
  D[3] = m3 * phi[2] * phi[0];

  D[4] = m3 * phi[1];
  D[5] = m3 * phi[0] * phi[1];
  D[6] = m3 * phi[1] * phi[1];
  D[7] = m3 * phi[2] * phi[1];

  D[ 8] = m3 * phi[2];
  D[ 9] = m3 * phi[0] * phi[2];
  D[10] = m3 * phi[1] * phi[2];
  D[11] = m3 * phi[2] * phi[2];
  D[1] += m;
  D[6] += m;
  D[11] += m;
}

void qmat_rotate(double* x2, const double* q, const double* x) {
  x2[0] = (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * x[0];
  x2[0] += 2 * (q[1] * q[2] - q[0]*q[3]) * x[1];
  x2[0] += 2 * (q[0] * q[2] + q[1]*q[3]) * x[2];

  x2[1] = (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]) * x[1];
  x2[1] += 2 * (q[1] * q[2] + q[0]*q[3]) * x[0];
  x2[1] += 2 * (q[2] * q[3] - q[0]*q[1]) * x[2];

  x2[2] = (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * x[2];
  x2[2] += 2 * (q[0] * q[1] + q[2]*q[3]) * x[1];
  x2[2] += 2 * (q[1] * q[3] - q[0]*q[2]) * x[0];
}

void qmat_drotate(double* D, const double* q, const double* x) {
  D[0] = 2*q[0]*x[0] + 2*q[2]*x[2] - 2*q[3]*x[1];
  D[1] = 2*q[3]*x[0] + 2*q[0]*x[1] - 2*q[1]*x[2];
  D[2] = 2*q[0]*x[2] + 2*q[1]*x[1] - 2*q[2]*x[0];

  D[3] =  2*q[1]*x[0] + 2*q[2]*x[1] + 2*q[3]*x[2];
  D[4] =  2*q[2]*x[0] - 2*q[0]*x[2] - 2*q[1]*x[1];
  D[5] =  2*q[3]*x[0] + 2*q[0]*x[1] - 2*q[1]*x[2];

  D[6] = 2*q[0]*x[2] + 2*q[1]*x[1] - 2*q[2]*x[0];
  D[7] = 2*q[1]*x[0] + 2*q[2]*x[1] + 2*q[3]*x[2];
  D[8] = 2*q[3]*x[1] - 2*q[0]*x[0] - 2*q[2]*x[2];

  D[ 9] = 2*q[1]*x[2] - 2*q[3]*x[0] - 2*q[0]*x[1];
  D[10] = 2*q[0]*x[0] + 2*q[2]*x[2] - 2*q[3]*x[1];
  D[11] = 2*q[1]*x[0] + 2*q[2]*x[1] + 2*q[3]*x[2];
}
