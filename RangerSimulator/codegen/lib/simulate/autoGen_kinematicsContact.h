/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * autoGen_kinematicsContact.h
 *
 * Code generation for function 'autoGen_kinematicsContact'
 *
 */

#ifndef __AUTOGEN_KINEMATICSCONTACT_H__
#define __AUTOGEN_KINEMATICSCONTACT_H__

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
extern void autoGen_kinematicsContact(double x, double y, double phi0, double
  phi1, double th0, double th1, double qc0, double qc1, double rc0, double rc1,
  double l, double p0c[2], double p1c[2]);

#endif

/* End of code generation (autoGen_kinematicsContact.h) */
