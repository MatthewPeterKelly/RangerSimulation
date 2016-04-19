/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * autoGen_dynamics_ds.h
 *
 * Code generation for function 'autoGen_dynamics_ds'
 *
 */

#ifndef __AUTOGEN_DYNAMICS_DS_H__
#define __AUTOGEN_DYNAMICS_DS_H__

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
extern void autoGen_dynamics_ds(double phi0, double phi1, double th0, double th1,
  double dphi0, double dphi1, double dth0, double dth1, double u0, double u1,
  double uHip, double fx, double fy, double qc0, double qc1, double rc1, double
  rc0, double ddp1c_x, double ddp1c_y, double ddp0c_x, double ddp0c_y, double g,
  double l, double c, double m, double I, double Ifoot, double b, double M_ds
  [100], double f_ds[10]);

#endif

/* End of code generation (autoGen_dynamics_ds.h) */
