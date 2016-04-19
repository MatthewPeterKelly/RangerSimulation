/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * dynamics_ds.c
 *
 * Code generation for function 'dynamics_ds'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "simulate.h"
#include "dynamics_ds.h"
#include "autoGen_dynamics_ds.h"

/* Function Definitions */

/*
 * function [ddz, F0, F1] = dynamics_ds(z,dz,u,f,qc,rc,ddc,dyn)
 */
void dynamics_ds(const double z[6], const double dz[6], const double u[3], const
                 double f[2], const double qc[2], const double rc[2], const
                 double ddc[4], double dyn_g, double dyn_l, double dyn_c, double
                 dyn_m, double dyn_I, double dyn_Ifoot, double dyn_b, double
                 ddz[6], double F0[2], double F1[2])
{
  double vars[10];
  double M_ds[100];
  signed char ipiv[10];
  int i1;
  int j;
  int c;
  int kAcol;
  int ix;
  double smax;
  int k;
  double s;
  int jy;
  int ijA;

  /*  [ddz, F0, F1] = dynamics_ds(z,dz,u,f,qc,rc,ddc,dyn) */
  /*  */
  /*  This function computes the double stance dynamics of Cornell Ranger, */
  /*  assuming that the acceleration of the contact points is known. */
  /*  */
  /*   */
  /*  z = [6, 1] = full configuration vector */
  /*  dz = [6, 1] = full configuration rate vector */
  /*  ddz = [6, 1] = full configuration acceleration vector */
  /*  	(1) = hip horizontal position */
  /*  	(2) = hip vertical position */
  /*  	(3) = outer foot absolute angle */
  /*  	(4) = inner foot absolute angle */
  /*  	(5) = outer leg absolute angle */
  /*  	(6) = inner leg absolute angle */
  /*   */
  /*  u = [3, 1] = motor torque vector */
  /*  	(1) = outer ankle joint torque (+leg, -foot) */
  /*  	(2) = inner ankle joint torque (+leg, -foot) */
  /*  	(3) = hip torque (+inner, -outer) */
  /*  */
  /*  f = [2, 1] = disturbance force applied at hip: */
  /*    (1) = fx = horizontal component */
  /*    (2) = fy = vertical component */
  /*   */
  /*  dyn = parameter struct: */
  /*  	.l = leg length */
  /*  	.g = acceleration due to gravity */
  /*  	.l = leg length (hip joint to foot joint) */
  /*  	.d = distance between foot joint and virtual center of foot */
  /*  	.r = radius of circular arc on foot */
  /*  	.c = distance along leg from hip to CoM */
  /*  	.w = distance off leg from main axis to CoM */
  /*  	.m = mass of each leg */
  /*  	.I = moment of inertia of the leg about its center of mass */
  /*  	.Ifoot = moment of inertia of the foot about the foot joint */
  /*   */
  /*  F = [4, 1] = contact force vector */
  /*  F = [F0; F1] = [outer; inner] contact forces */
  /*  	(1) = horizontal component */
  /*  	(2) = vertical component */
  /*   */
  /*  Angle Convention: */
  /*  	- All angles are in absolute reference frame */
  /*      - zero corresponds to: */
  /*      	(legs) = ankle joint directly below hip joint */
  /*          (feet) = ankle joint directly below virtual foot center */
  /* 'dynamics_ds:50' nState = 6; */
  /* 'dynamics_ds:51' nContact = 4; */
  /*  x = z(1); */
  /*  y = z(2); */
  /* 'dynamics_ds:55' phi0 = z(3); */
  /* 'dynamics_ds:56' phi1 = z(4); */
  /* 'dynamics_ds:57' th0 = z(5); */
  /* 'dynamics_ds:58' th1 = z(6); */
  /*  dx = dz(1); */
  /*  dy = dz(2); */
  /* 'dynamics_ds:62' dphi0 = dz(3); */
  /* 'dynamics_ds:63' dphi1 = dz(4); */
  /* 'dynamics_ds:64' dth0 = dz(5); */
  /* 'dynamics_ds:65' dth1 = dz(6); */
  /* 'dynamics_ds:67' u0 = u(1); */
  /* 'dynamics_ds:68' u1 = u(2); */
  /* 'dynamics_ds:69' uHip = u(3); */
  /* 'dynamics_ds:71' fx = f(1); */
  /* 'dynamics_ds:72' fy = f(2); */
  /*  Contact points for each foot */
  /* 'dynamics_ds:75' qc0 = qc(1); */
  /* 'dynamics_ds:76' qc1 = qc(2); */
  /* 'dynamics_ds:77' rc0 = rc(1); */
  /* 'dynamics_ds:78' rc1 = rc(2); */
  /*  Acceleration of the contact points: */
  /* 'dynamics_ds:81' ddp0c_x = ddc(1); */
  /* 'dynamics_ds:82' ddp0c_y = ddc(2); */
  /* 'dynamics_ds:83' ddp1c_x = ddc(3); */
  /* 'dynamics_ds:84' ddp1c_y = ddc(4); */
  /* %%% call to dynamics */
  /* 'dynamics_ds:87' [M_ds,f_ds] = autoGen_dynamics_ds(... */
  /* 'dynamics_ds:88'     phi0,phi1,th0,th1,... */
  /* 'dynamics_ds:89'     dphi0,dphi1,dth0,dth1,... */
  /* 'dynamics_ds:90'     u0,u1,uHip,... */
  /* 'dynamics_ds:91'     fx, fy,... */
  /* 'dynamics_ds:92'     qc0,qc1,rc1,rc0,... */
  /* 'dynamics_ds:93'     ddp1c_x,ddp1c_y,ddp0c_x,ddp0c_y,... */
  /* 'dynamics_ds:94'     dyn.g,dyn.l,dyn.c,dyn.m,dyn.I,dyn.Ifoot,dyn.b); */
  autoGen_dynamics_ds(z[2], z[3], z[4], z[5], dz[2], dz[3], dz[4], dz[5], u[0],
                      u[1], u[2], f[0], f[1], qc[0], qc[1], rc[1], rc[0], ddc[2],
                      ddc[3], ddc[0], ddc[1], dyn_g, dyn_l, dyn_c, dyn_m, dyn_I,
                      dyn_Ifoot, dyn_b, M_ds, vars);

  /* 'dynamics_ds:95' vars = M_ds\f_ds; */
  for (i1 = 0; i1 < 10; i1++) {
    ipiv[i1] = (signed char)(1 + i1);
  }

  for (j = 0; j < 9; j++) {
    c = j * 11;
    kAcol = 0;
    ix = c;
    smax = fabs(M_ds[c]);
    for (k = 1; k + 1 <= 10 - j; k++) {
      ix++;
      s = fabs(M_ds[ix]);
      if (s > smax) {
        kAcol = k;
        smax = s;
      }
    }

    if (M_ds[c + kAcol] != 0.0) {
      if (kAcol != 0) {
        ipiv[j] = (signed char)((j + kAcol) + 1);
        ix = j;
        kAcol += j;
        for (k = 0; k < 10; k++) {
          smax = M_ds[ix];
          M_ds[ix] = M_ds[kAcol];
          M_ds[kAcol] = smax;
          ix += 10;
          kAcol += 10;
        }
      }

      i1 = (c - j) + 10;
      for (jy = c + 1; jy + 1 <= i1; jy++) {
        M_ds[jy] /= M_ds[c];
      }
    }

    kAcol = c;
    jy = c + 10;
    for (k = 1; k <= 9 - j; k++) {
      smax = M_ds[jy];
      if (M_ds[jy] != 0.0) {
        ix = c + 1;
        i1 = (kAcol - j) + 20;
        for (ijA = 11 + kAcol; ijA + 1 <= i1; ijA++) {
          M_ds[ijA] += M_ds[ix] * -smax;
          ix++;
        }
      }

      jy += 10;
      kAcol += 10;
    }

    if (ipiv[j] != j + 1) {
      smax = vars[j];
      vars[j] = vars[ipiv[j] - 1];
      vars[ipiv[j] - 1] = smax;
    }
  }

  for (k = 0; k < 10; k++) {
    kAcol = 10 * k;
    if (vars[k] != 0.0) {
      for (jy = k + 1; jy + 1 < 11; jy++) {
        vars[jy] -= vars[k] * M_ds[jy + kAcol];
      }
    }
  }

  for (k = 9; k >= 0; k += -1) {
    kAcol = 10 * k;
    if (vars[k] != 0.0) {
      vars[k] /= M_ds[k + kAcol];
      for (jy = 0; jy + 1 <= k; jy++) {
        vars[jy] -= vars[k] * M_ds[jy + kAcol];
      }
    }
  }

  /* %%% Unpack the result: */
  /* 'dynamics_ds:98' ddz = vars(1:nState); */
  for (jy = 0; jy < 6; jy++) {
    ddz[jy] = vars[jy];
  }

  /* 'dynamics_ds:99' F = vars( (nState+1) : (nState+nContact) ); */
  /* 'dynamics_ds:101' F0 = F(1:2); */
  /* 'dynamics_ds:102' F1 = F(3:4); */
  for (i1 = 0; i1 < 2; i1++) {
    F0[i1] = vars[6 + i1];
    F1[i1] = vars[i1 + 8];
  }
}

/* End of code generation (dynamics_ds.c) */
