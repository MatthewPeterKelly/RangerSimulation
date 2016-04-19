/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * simulate_initialize.c
 *
 * Code generation for function 'simulate_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "simulate.h"
#include "simulate_initialize.h"
#include "simulate_data.h"

/* Function Definitions */
void simulate_initialize(void)
{
  rt_InitInfAndNaN(8U);
  omp_init_nest_lock(&emlrtNestLockGlobal);
}

/* End of code generation (simulate_initialize.c) */
