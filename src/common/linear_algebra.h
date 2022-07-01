#pragma once

int sub2ind(int i, int j, int rows);

void rexquad_SetZero(double n, double m, double* mat);

void rexquad_SetConstant(double n, double m, double* mat, double alpha);

void rexquad_SetIdentity(double n, double* mat, double alpha);

void rexquad_PrintMatrix(double n, double m, const double* mat, int precision);

double rexquad_NormSquaredDifference(double n, const double* a, const double* b);
