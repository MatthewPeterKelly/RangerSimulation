/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * motorModel.h
 *
 * Code generation for function 'motorModel'
 *
 */

#ifndef __MOTORMODEL_H__
#define __MOTORMODEL_H__

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
extern void motorModel(const double x[12], const double iRef[3], const double
  cp[3], const double cd[3], double model_motor_ank_R, double model_motor_ank_Vc,
  double model_motor_ank_K, double model_motor_ank_G, double model_motor_ank_c1,
  double model_motor_ank_c0, double model_motor_ank_mu, double
  model_motor_ank_Imax, double model_motor_ank_alpha, double
  model_motor_ank_xSpring, double model_motor_ank_kSpring, const struct2_T
  *model_motor_hip, double model_motor_Phi, double u[3], double b_power[3],
  double current[3]);

#endif

/* End of code generation (motorModel.h) */
