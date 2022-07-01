#pragma once

void qmat_inv(double* qinv, const double* q);
void qmat_quat2rotmat(double* Q, const double* q);
void qmat_skewmat(double* S, const double* x);
void qmat_lmat(double* L, const double* q);
void qmat_rmat(double* R, const double* q);
void qmat_gmat(double* G, double* q);
void qmat_cay(double* q, const double* phi);
void qmat_icay(double* phi, const double* q);
void qmat_dcay(double* D, const double* phi);
void qmat_rotate(double* x2, const double* q, const double* x);
void qmat_drotate(double* D, const double* q, const double* x);
