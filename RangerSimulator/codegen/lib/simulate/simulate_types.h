/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * simulate_types.h
 *
 * Code generation for function 'simulate'
 *
 */

#ifndef __SIMULATE_TYPES_H__
#define __SIMULATE_TYPES_H__

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  double g;
  double l;
  double d;
  double r;
  double c;
  double m;
  double I;
  double Ifoot;
  double b;
  double Phi;
  double ground[6];
  double dt;
  double cstWn;
  double cstXi;
  double maxStepDuration;
  double minStepDuration;
  double nDataPerStep;
} struct0_T;

#endif                                 /*typedef_struct0_T*/

#ifndef typedef_struct2_T
#define typedef_struct2_T

typedef struct {
  double R;
  double Vc;
  double K;
  double G;
  double c1;
  double c0;
  double mu;
  double Imax;
  double alpha;
  double xSpring;
  double kSpring;
} struct2_T;

#endif                                 /*typedef_struct2_T*/

#ifndef typedef_struct1_T
#define typedef_struct1_T

typedef struct {
  struct2_T ank;
  struct2_T hip;
  double overheadPower;
  double Phi;
} struct1_T;

#endif                                 /*typedef_struct1_T*/
#endif

/* End of code generation (simulate_types.h) */
