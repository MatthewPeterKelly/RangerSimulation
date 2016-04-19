/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "simulate.h"
#include "mldivide.h"

/* Function Definitions */

/*
 *
 */
void mldivide(const double A[64], double B[8])
{
  double b_A[64];
  signed char ipiv[8];
  int i3;
  int j;
  int c;
  int kAcol;
  int ix;
  double smax;
  int k;
  double s;
  int jy;
  int ijA;
  memcpy(&b_A[0], &A[0], sizeof(double) << 6);
  for (i3 = 0; i3 < 8; i3++) {
    ipiv[i3] = (signed char)(1 + i3);
  }

  for (j = 0; j < 7; j++) {
    c = j * 9;
    kAcol = 0;
    ix = c;
    smax = fabs(b_A[c]);
    for (k = 1; k + 1 <= 8 - j; k++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > smax) {
        kAcol = k;
        smax = s;
      }
    }

    if (b_A[c + kAcol] != 0.0) {
      if (kAcol != 0) {
        ipiv[j] = (signed char)((j + kAcol) + 1);
        ix = j;
        kAcol += j;
        for (k = 0; k < 8; k++) {
          smax = b_A[ix];
          b_A[ix] = b_A[kAcol];
          b_A[kAcol] = smax;
          ix += 8;
          kAcol += 8;
        }
      }

      i3 = (c - j) + 8;
      for (jy = c + 1; jy + 1 <= i3; jy++) {
        b_A[jy] /= b_A[c];
      }
    }

    kAcol = c;
    jy = c + 8;
    for (k = 1; k <= 7 - j; k++) {
      smax = b_A[jy];
      if (b_A[jy] != 0.0) {
        ix = c + 1;
        i3 = (kAcol - j) + 16;
        for (ijA = 9 + kAcol; ijA + 1 <= i3; ijA++) {
          b_A[ijA] += b_A[ix] * -smax;
          ix++;
        }
      }

      jy += 8;
      kAcol += 8;
    }

    if (ipiv[j] != j + 1) {
      smax = B[j];
      B[j] = B[ipiv[j] - 1];
      B[ipiv[j] - 1] = smax;
    }
  }

  for (k = 0; k < 8; k++) {
    kAcol = k << 3;
    if (B[k] != 0.0) {
      for (jy = k + 1; jy + 1 < 9; jy++) {
        B[jy] -= B[k] * b_A[jy + kAcol];
      }
    }
  }

  for (k = 7; k >= 0; k += -1) {
    kAcol = k << 3;
    if (B[k] != 0.0) {
      B[k] /= b_A[k + kAcol];
      for (jy = 0; jy + 1 <= k; jy++) {
        B[jy] -= B[k] * b_A[jy + kAcol];
      }
    }
  }
}

/* End of code generation (mldivide.c) */
