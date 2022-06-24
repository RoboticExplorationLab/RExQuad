#include "linear_algebra.h"

#include <string.h>
#include <stdio.h>

int sub2ind(int i, int j, int rows) { return i + rows * j; }

void rexquad_SetZero(double n, double m, double* mat) {
  memset((void*)mat, 0, n * m * sizeof(double));
}

void rexquad_SetConstant(double n, double m, double* mat, double alpha) {
  for (int j = 0; j < m; ++j) {
    for (int i = 0; i < n; ++i) {
      mat[sub2ind(i,j,n)] = alpha;
    }
  }
}

void rexquad_SetIdentity(double n, double* mat, double alpha) {
  memset((void*)mat, 0, n * n * sizeof(double));
  for (int i = 0; i < n; ++i) {
    int k = sub2ind(i, i, n);
    mat[k] = alpha;
  }
}

void rexquad_PrintMatrix(double n, double m, const double* mat, int precision) {
  for (int row = 0; row < n; ++row) {
    for (int col = 0; col < m; ++col) {
      printf("% 6.*g ", precision, mat[sub2ind(row, col, n)]);
    }
    printf("\n");
  }
}

double rexquad_NormSquaredDifference(double n, const double* a, const double* b) {
  double err = 0.0;
  for (int i = 0; i < n; ++i) {
    double d = a[i] - b[i];
    err += d*d;
  }
  return err;
}