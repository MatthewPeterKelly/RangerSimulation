/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * dynamics_fl.c
 *
 * Code generation for function 'dynamics_fl'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "simulate.h"
#include "dynamics_fl.h"

/* Function Definitions */

/*
 * function ddz = dynamics_fl(z,dz,u,f,dyn)
 */
void dynamics_fl(const double z[6], const double dz[6], const double u[3], const
                 double f[2], double dyn_g, double dyn_c, double dyn_m, double
                 dyn_I, double dyn_Ifoot, double ddz[6])
{
  double t2;
  double t3;
  double t4;
  double t5;
  double t7;
  double x[36];
  double M_fl[36];
  double t8;
  double t9;
  signed char ipiv[6];
  int i2;
  int j;
  int c;
  int kAcol;
  int ix;
  double smax;
  int k;
  double s;
  int jy;
  int ijA;

  /*  ddz = dynamics_fl(z,dz,u,f,qc,p) */
  /*  */
  /*  This function computes the dynamics of the Cornell Ranger, assuming that */
  /*  both feet are in the air. It also computes acceleration of the contact */
  /*  point on each foot. */
  /*  */
  /*  INPUTS: */
  /*  z = [6, 1] = full configuration vector */
  /*  dz = [6, 1] = full configuration rate vector */
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
  /*    qc = [2 x 1] contact point angle relative to foot */
  /*  */
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
  /*    p = parameter struct: */
  /*     .g = acceleration due to gravity */
  /*     .l = leg length (hip joint to foot joint) */
  /*     .d = distance between foot joint and virtual center of foot */
  /*     .r = radius of circular arc on foot */
  /*     .c = distance along leg from hip to CoM */
  /*     .w = distance off leg from main axis to CoM */
  /*     .m = mass of each leg */
  /*     .I = moment of inertia of the leg about its center of mass */
  /*     .Ifoot = moment of inertia of the foot about the foot joint */
  /*  */
  /*  OUTPUTS: */
  /*    ddz = [6 x 1] angle acceleration vector */
  /*    ddc0 = [2 x 1] acceleration vector for outer foot contact */
  /*    ddc1 = [2 x 1] acceleration vector for inner foot contact */
  /*   */
  /*  NOTES: */
  /*   */
  /*   */
  /*  f = [4, 1] = contact force vector */
  /*  f = [f0; f1] = [outer; inner] contact forces */
  /*  	(1) = horizontal component */
  /*  	(2) = vertical component */
  /*   */
  /*  Angle Convention: */
  /*  	- All angles are in absolute reference frame */
  /*      - zero corresponds to: */
  /*      	(legs) = ankle joint directly below hip joint */
  /*          (feet) = ankle joint directly below virtual foot center */
  /*     */
  /*    -->  ** Even though there are no contact forces, we still need */
  /*    "contact" points for the angular momentum balance equations. For the */
  /*    purposes of this function, any points will work, provided that they are */
  /*    distinct from both each and the locations of the hip and ankle joints. */
  /*  */
  /*  x = z(1); */
  /*  y = z(2); */
  /*  phi0 = z(3); */
  /*  phi1 = z(4); */
  /* 'dynamics_fl:82' th0 = z(5); */
  /* 'dynamics_fl:83' th1 = z(6); */
  /*  dx = dz(1); */
  /*  dy = dz(2); */
  /*  dphi0 = dz(3); */
  /*  dphi1 = dz(4); */
  /* 'dynamics_fl:89' dth0 = dz(5); */
  /* 'dynamics_fl:90' dth1 = dz(6); */
  /* 'dynamics_fl:92' u0 = u(1); */
  /* 'dynamics_fl:93' u1 = u(2); */
  /* 'dynamics_fl:94' uHip = u(3); */
  /* 'dynamics_fl:96' fx = f(1); */
  /* 'dynamics_fl:97' fy = f(2); */
  /* Call the flight dynamics */
  /* 'dynamics_fl:102' [M_fl,f_fl] = autoGen_dynamics_fl(... */
  /* 'dynamics_fl:103'     th0,th1,... */
  /* 'dynamics_fl:104'     dth0,dth1,... */
  /* 'dynamics_fl:105'     u0,u1,uHip,... */
  /* 'dynamics_fl:106'     fx, fy,... */
  /* 'dynamics_fl:107'     dyn.g, dyn.c, dyn.m, dyn.I, dyn.Ifoot); */
  /* AUTOGEN_DYNAMICS_FL */
  /*     [M_FL,F_FL] = AUTOGEN_DYNAMICS_FL(TH0,TH1,DTH0,DTH1,U0,U1,UHIP,FX,FY,G,C,M,I,IFOOT) */
  /*     This function was generated by the Symbolic Math Toolbox version 6.3. */
  /*     30-Jan-2016 13:22:41 */
  /* 'autoGen_dynamics_fl:8' t2 = cos(th0); */
  t2 = cos(z[4]);

  /* 'autoGen_dynamics_fl:9' t3 = sin(th0); */
  t3 = sin(z[4]);

  /* 'autoGen_dynamics_fl:10' t4 = cos(th1); */
  t4 = cos(z[5]);

  /* 'autoGen_dynamics_fl:11' t5 = sin(th1); */
  t5 = sin(z[5]);

  /* 'autoGen_dynamics_fl:12' t6 = c.^2; */
  /* 'autoGen_dynamics_fl:13' t7 = -I-m.*t6; */
  t7 = -dyn_I - dyn_m * (dyn_c * dyn_c);

  /* 'autoGen_dynamics_fl:14' M_fl = reshape([m.*-2.0,0.0,-c.*m.*t2,-c.*m.*t4,0.0,0.0,0.0,m.*-2.0,-c.*m.*t3,-c.*m.*t5,0.0,0.0,0.0,0.0,-Ifoot,0.0,-Ifoot,0.0,0.0,0.0,0.0,-Ifoot,0.0,-Ifoot,-c.*m.*t2,-c.*m.*t3,t7,0.0,0.0,0.0,-c.*m.*t4,-c.*m.*t5,0.0,t7,0.0,0.0],[6,6]); */
  x[0] = dyn_m * -2.0;
  x[1] = 0.0;
  x[2] = -dyn_c * dyn_m * t2;
  x[3] = -dyn_c * dyn_m * t4;
  x[4] = 0.0;
  x[5] = 0.0;
  x[6] = 0.0;
  x[7] = dyn_m * -2.0;
  x[8] = -dyn_c * dyn_m * t3;
  x[9] = -dyn_c * dyn_m * t5;
  x[10] = 0.0;
  x[11] = 0.0;
  x[12] = 0.0;
  x[13] = 0.0;
  x[14] = -dyn_Ifoot;
  x[15] = 0.0;
  x[16] = -dyn_Ifoot;
  x[17] = 0.0;
  x[18] = 0.0;
  x[19] = 0.0;
  x[20] = 0.0;
  x[21] = -dyn_Ifoot;
  x[22] = 0.0;
  x[23] = -dyn_Ifoot;
  x[24] = -dyn_c * dyn_m * t2;
  x[25] = -dyn_c * dyn_m * t3;
  x[26] = t7;
  x[27] = 0.0;
  x[28] = 0.0;
  x[29] = 0.0;
  x[30] = -dyn_c * dyn_m * t4;
  x[31] = -dyn_c * dyn_m * t5;
  x[32] = 0.0;
  x[33] = t7;
  x[34] = 0.0;
  x[35] = 0.0;
  memcpy(&M_fl[0], &x[0], 36U * sizeof(double));

  /* 'autoGen_dynamics_fl:15' if nargout > 1 */
  /* 'autoGen_dynamics_fl:16' t8 = dth0.^2; */
  t8 = dz[4] * dz[4];

  /* 'autoGen_dynamics_fl:17' t9 = dth1.^2; */
  t9 = dz[5] * dz[5];

  /* 'autoGen_dynamics_fl:18' f_fl = [-fx-m.*(c.*t3.*t8.*(1.0./2.0)+c.*t5.*t9.*(1.0./2.0)).*2.0;-fy+g.*m.*2.0+c.*m.*t2.*t8+c.*m.*t4.*t9;uHip+c.*g.*m.*t3;-uHip+c.*g.*m.*t5;u0;u1]; */
  /* 'dynamics_fl:109' ddz = M_fl\f_fl; */
  for (i2 = 0; i2 < 6; i2++) {
    ipiv[i2] = (signed char)(1 + i2);
  }

  ddz[0] = -f[0] - dyn_m * (dyn_c * t3 * t8 * 0.5 + dyn_c * t5 * t9 * 0.5) * 2.0;
  ddz[1] = ((-f[1] + dyn_g * dyn_m * 2.0) + dyn_c * dyn_m * t2 * t8) + dyn_c *
    dyn_m * t4 * t9;
  ddz[2] = u[2] + dyn_c * dyn_g * dyn_m * t3;
  ddz[3] = -u[2] + dyn_c * dyn_g * dyn_m * t5;
  ddz[4] = u[0];
  ddz[5] = u[1];
  for (j = 0; j < 5; j++) {
    c = j * 7;
    kAcol = 0;
    ix = c;
    smax = fabs(M_fl[c]);
    for (k = 1; k + 1 <= 6 - j; k++) {
      ix++;
      s = fabs(M_fl[ix]);
      if (s > smax) {
        kAcol = k;
        smax = s;
      }
    }

    if (M_fl[c + kAcol] != 0.0) {
      if (kAcol != 0) {
        ipiv[j] = (signed char)((j + kAcol) + 1);
        ix = j;
        kAcol += j;
        for (k = 0; k < 6; k++) {
          smax = M_fl[ix];
          M_fl[ix] = M_fl[kAcol];
          M_fl[kAcol] = smax;
          ix += 6;
          kAcol += 6;
        }
      }

      i2 = (c - j) + 6;
      for (jy = c + 1; jy + 1 <= i2; jy++) {
        M_fl[jy] /= M_fl[c];
      }
    }

    kAcol = c;
    jy = c + 6;
    for (k = 1; k <= 5 - j; k++) {
      smax = M_fl[jy];
      if (M_fl[jy] != 0.0) {
        ix = c + 1;
        i2 = (kAcol - j) + 12;
        for (ijA = 7 + kAcol; ijA + 1 <= i2; ijA++) {
          M_fl[ijA] += M_fl[ix] * -smax;
          ix++;
        }
      }

      jy += 6;
      kAcol += 6;
    }

    if (ipiv[j] != j + 1) {
      smax = ddz[j];
      ddz[j] = ddz[ipiv[j] - 1];
      ddz[ipiv[j] - 1] = smax;
    }
  }

  for (k = 0; k < 6; k++) {
    kAcol = 6 * k;
    if (ddz[k] != 0.0) {
      for (jy = k + 1; jy + 1 < 7; jy++) {
        ddz[jy] -= ddz[k] * M_fl[jy + kAcol];
      }
    }
  }

  for (k = 5; k >= 0; k += -1) {
    kAcol = 6 * k;
    if (ddz[k] != 0.0) {
      ddz[k] /= M_fl[k + kAcol];
      for (jy = 0; jy + 1 <= k; jy++) {
        ddz[jy] -= ddz[k] * M_fl[jy + kAcol];
      }
    }
  }
}

/* End of code generation (dynamics_fl.c) */
