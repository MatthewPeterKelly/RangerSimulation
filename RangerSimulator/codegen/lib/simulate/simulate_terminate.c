/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * simulate_terminate.c
 *
 * Code generation for function 'simulate_terminate'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "simulate.h"
#include "simulate_terminate.h"
#include "simulate_data.h"

/* Function Definitions */
void simulate_terminate(void)
{
  omp_destroy_nest_lock(&emlrtNestLockGlobal);
}

/* End of code generation (simulate_terminate.c) */
