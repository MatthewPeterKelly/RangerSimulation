/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * dynamics_fl.h
 *
 * Code generation for function 'dynamics_fl'
 *
 */

#ifndef __DYNAMICS_FL_H__
#define __DYNAMICS_FL_H__

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
extern void dynamics_fl(const double z[6], const double dz[6], const double u[3],
  const double f[2], double dyn_g, double dyn_c, double dyn_m, double dyn_I,
  double dyn_Ifoot, double ddz[6]);

#endif

/* End of code generation (dynamics_fl.h) */
