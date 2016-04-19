/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * power.c
 *
 * Code generation for function 'power'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "simulate.h"
#include "power.h"

/* Function Definitions */

/*
 *
 */
void power(const double a_data[], const int a_size[2], double y_data[], int
           y_size[2])
{
  int loop_ub;
  int i0;
  double x_data[1];
  int k;
  int b_k;
  loop_ub = a_size[0] * a_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x_data[i0] = a_data[i0];
  }

  y_size[0] = 1;
  y_size[1] = (signed char)a_size[1];
  loop_ub = a_size[1];

#pragma omp parallel for \
 num_threads(omp_get_max_threads()) \
 private(b_k)

  for (k = 1; k <= loop_ub; k++) {
    b_k = k;
    y_data[b_k - 1] = x_data[b_k - 1] * x_data[b_k - 1];
  }
}

/* End of code generation (power.c) */
