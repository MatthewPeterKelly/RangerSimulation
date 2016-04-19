/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * simulate.h
 *
 * Code generation for function 'simulate'
 *
 */

#ifndef __SIMULATE_H__
#define __SIMULATE_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "omp.h"
#include "simulate_types.h"

/* Function Declarations */
extern void simulate(const double state[12], const double ir[3], const double
                     cp[3], const double cd[3], const struct0_T *dyn, const
                     struct1_T *motor, const double dist[2], double stateNext[12],
                     double f[4], double c[4], boolean_T k[2], double torque[3],
                     double b_power[3], double current[3]);

#endif

/* End of code generation (simulate.h) */
