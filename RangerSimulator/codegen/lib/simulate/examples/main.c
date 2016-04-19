/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.c
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include files */
#include "rt_nonfinite.h"
#include "simulate.h"
#include "main.h"
#include "simulate_terminate.h"
#include "simulate_initialize.h"

/* Function Declarations */
static void argInit_12x1_real_T(double result[12]);
static void argInit_1x6_real_T(double result[6]);
static void argInit_2x1_real_T(double result[2]);
static void argInit_3x1_real_T(double result[3]);
static double argInit_real_T(void);
static void argInit_struct0_T(struct0_T *result);
static void argInit_struct1_T(struct1_T *result);
static void argInit_struct2_T(struct2_T *result);
static void main_simulate(void);

/* Function Definitions */
static void argInit_12x1_real_T(double result[12])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 12; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static void argInit_1x6_real_T(double result[6])
{
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 6; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

static void argInit_2x1_real_T(double result[2])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 2; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static void argInit_3x1_real_T(double result[3])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T(void)
{
  return 0.0;
}

static void argInit_struct0_T(struct0_T *result)
{
  /* Set the value of each structure field.
     Change this value to the value that the application requires. */
  result->g = argInit_real_T();
  result->l = argInit_real_T();
  result->d = argInit_real_T();
  result->r = argInit_real_T();
  result->c = argInit_real_T();
  result->m = argInit_real_T();
  result->I = argInit_real_T();
  result->Ifoot = argInit_real_T();
  result->b = argInit_real_T();
  result->Phi = argInit_real_T();
  argInit_1x6_real_T(result->ground);
  result->dt = argInit_real_T();
  result->cstWn = argInit_real_T();
  result->cstXi = argInit_real_T();
  result->maxStepDuration = argInit_real_T();
  result->minStepDuration = argInit_real_T();
  result->nDataPerStep = argInit_real_T();
}

static void argInit_struct1_T(struct1_T *result)
{
  /* Set the value of each structure field.
     Change this value to the value that the application requires. */
  argInit_struct2_T(&result->ank);
  argInit_struct2_T(&result->hip);
  result->overheadPower = argInit_real_T();
  result->Phi = argInit_real_T();
}

static void argInit_struct2_T(struct2_T *result)
{
  /* Set the value of each structure field.
     Change this value to the value that the application requires. */
  result->R = argInit_real_T();
  result->Vc = argInit_real_T();
  result->K = argInit_real_T();
  result->G = argInit_real_T();
  result->c1 = argInit_real_T();
  result->c0 = argInit_real_T();
  result->mu = argInit_real_T();
  result->Imax = argInit_real_T();
  result->alpha = argInit_real_T();
  result->xSpring = argInit_real_T();
  result->kSpring = argInit_real_T();
}

static void main_simulate(void)
{
  double dv4[12];
  double dv5[3];
  double dv6[3];
  double dv7[3];
  struct0_T r0;
  struct1_T r1;
  double dv8[2];
  double current[3];
  double b_power[3];
  double torque[3];
  boolean_T k[2];
  double c[4];
  double f[4];
  double stateNext[12];

  /* Initialize function 'simulate' input arguments. */
  /* Initialize function input argument 'state'. */
  /* Initialize function input argument 'ir'. */
  /* Initialize function input argument 'cp'. */
  /* Initialize function input argument 'cd'. */
  /* Initialize function input argument 'dyn'. */
  /* Initialize function input argument 'motor'. */
  /* Initialize function input argument 'dist'. */
  /* Call the entry-point 'simulate'. */
  argInit_12x1_real_T(dv4);
  argInit_3x1_real_T(dv5);
  argInit_3x1_real_T(dv6);
  argInit_3x1_real_T(dv7);
  argInit_struct0_T(&r0);
  argInit_struct1_T(&r1);
  argInit_2x1_real_T(dv8);
  simulate(dv4, dv5, dv6, dv7, &r0, &r1, dv8, stateNext, f, c, k, torque,
           b_power, current);
}

int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* Initialize the application.
     You do not need to do this more than one time. */
  simulate_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_simulate();

  /* Terminate the application.
     You do not need to do this more than one time. */
  simulate_terminate();
  return 0;
}

/* End of code generation (main.c) */
