/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * footModel.c
 *
 * Code generation for function 'footModel'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "simulate.h"
#include "footModel.h"
#include "power.h"

/* Function Declarations */
static double rt_roundd_snf(double u);

/* Function Definitions */
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * function r = footModel(th)
 */
double footModel(double th)
{
  double r;
  static double dv0[7] = { 0.0, -0.561694831717942, -0.286174659654303,
    1.250200489359516, 1.46657216777903, 2.0344439357957032, 0.0 };

  boolean_T positiveInput;
  double lambda;
  double unnamed_idx_0;
  int i;
  int low_i;
  int low_ip1;
  int high_i;
  int mid_i;
  static const double dv1[7] = { -1.570796326794897, -0.561694831717942,
    -0.286174659654303, 1.250200489359516, 1.46657216777903, 2.0344439357957032,
    4.71238898038469 };

  double dt;
  int tt_size[2];
  double tt_data[1];
  double dp[7];
  double ddp[7];
  static const double a[7] = { 0.001120783999919, 0.07767946981193,
    -0.012561666565018, 0.094066800804457, -0.220297541073049,
    -0.030857935325635, 0.001120783999919 };

  static const double b_a[7] = { 0.003819397812255, 0.263409043236919,
    0.047665562305856, 0.146386617183001, 1.0895331247001521, 0.019752990773765,
    0.003819397812255 };

  int t2_size[2];
  double t2_data[1];
  int t3_size[2];
  double t3_data[1];
  int t4_size[2];
  double t4_data[1];
  double t5_data[1];
  double t6_data[1];
  static const double dv2[7] = { 0.026429428615028, 0.049616197502626,
    0.061753405058648, 0.105377371861968, 0.088455248234482, 0.040233205117705,
    0.026429428615028 };

  double t8;
  double t10;
  double t13;
  double t15;
  dv0[0U] = rtMinusInf;
  dv0[6U] = rtInf;

  /*  r = footModel(th) */
  /*  */
  /*  This function computes the distance (r) from the ankle joint to the edge */
  /*  of the foot, given some angle (th) in the frame of the foot. */
  /*  */
  /*  INPUTS: */
  /*    th = [1, n] = vector of query angles, measured in the frame of the */
  /*    foot, where 0 corresponds to the -j axis. For example: */
  /*  */
  /*            [y,x] = pol2cart(th,r); y = -y; */
  /*  */
  /*  OUTPUTS: */
  /*    r = [1, n] = radius (edge of foot to axle) in meters */
  /*  */
  /*  NOTES: */
  /*    --> The foot is modeled using a piecewise 5th-order polynomial that has */
  /*    continuous second derivatives across each knot point, and is periodic. */
  /*  */
  /* %%% Parameters computed in MAIN_circleModel.m */
  /*  Knot points for the piecewise 5th-order polynomial */
  /* 'footModel:24' Th = [... */
  /* 'footModel:25'       -1.570796326794897; */
  /* 'footModel:26'   -0.561694831717942; */
  /* 'footModel:27'   -0.286174659654303; */
  /* 'footModel:28'    1.250200489359516; */
  /* 'footModel:29'    1.466572167779030; */
  /* 'footModel:30'    2.034443935795703; */
  /* 'footModel:31'    4.712388980384690]; */
  /*  Function value, slope, and curvature at knot points: */
  /*  [r(Th), dr(Th), ddr(Th)]; */
  /* 'footModel:35' P = [... */
  /* 'footModel:36'    0.026429428615028   0.001120783999919   0.003819397812255; */
  /* 'footModel:37'    0.049616197502626   0.077679469811930   0.263409043236919; */
  /* 'footModel:38'    0.061753405058648  -0.012561666565018   0.047665562305856; */
  /* 'footModel:39'    0.105377371861968   0.094066800804457   0.146386617183001; */
  /* 'footModel:40'    0.088455248234482  -0.220297541073049   1.089533124700152; */
  /* 'footModel:41'    0.040233205117705  -0.030857935325635   0.019752990773765; */
  /* 'footModel:42'    0.026429428615028   0.001120783999919   0.003819397812255]; */
  /*  Map th to the appropriate domain: */
  /* 'footModel:45' thWrap = wrapTo2Pi(th - Th(1)) + Th(1); */
  /* wrapTo2Pi Wrap angle in radians to [0 2*pi] */
  /*  */
  /*    lambdaWrapped = wrapTo2Pi(LAMBDA) wraps angles in LAMBDA, in radians, */
  /*    to the interval [0 2*pi] such that zero maps to zero and 2*pi maps */
  /*    to 2*pi. (In general, positive multiples of 2*pi map to 2*pi and */
  /*    negative multiples of 2*pi map to zero.) */
  /*  */
  /*    See also wrapToPi, wrapTo180, wrapTo360. */
  /*  Copyright 2007-2008 The MathWorks, Inc. */
  /* 'wrapTo2Pi:13' positiveInput = (lambda > 0); */
  positiveInput = (th - -1.570796326794897 > 0.0);

  /* 'wrapTo2Pi:14' lambda = mod(lambda, 2*pi); */
  lambda = (th - -1.570796326794897) / 6.2831853071795862;
  if (fabs(lambda - rt_roundd_snf(lambda)) <= 2.2204460492503131E-16 * fabs
      (lambda)) {
    lambda = 0.0;
  } else {
    lambda = (lambda - floor(lambda)) * 6.2831853071795862;
  }

  /* 'wrapTo2Pi:15' lambda((lambda == 0) & positiveInput) = 2*pi; */
  unnamed_idx_0 = lambda;
  for (i = 0; i < 1; i++) {
    if ((lambda == 0.0) && positiveInput) {
      unnamed_idx_0 = 6.2831853071795862;
    }
  }

  /*  Evaluate the piecewise 5th-order curve: */
  /* 'footModel:48' r = pwPoly5(Th',P',thWrap); */
  /*  [x,dx,ddx] = pwPoly5(T,P,t) */
  /*  */
  /*  This function computes a piece-wise 5th-order polynomial that */
  /*  interpolates the points in P, which contain function value, slope, and */
  /*  curvature information. */
  /*  */
  /*  INPUTS: */
  /*    T = [1, nKnot] vector of the time at each knot point */
  /*    P = [3, nKnot] = [x;dx;ddx] = function data at each knot point */
  /*    t = [1, nTime] = vector of desired interpolation times */
  /*  */
  /*  OUTPUTS: */
  /*    x = [1, nTime] = function value */
  /*    dx = [1, nTime] = first time derivative of x */
  /*    ddx = [1, nTime] = second time derivative of x */
  /*  */
  /*  ASSUME: */
  /*    --> T is monotonicall increasing: diff(T) > 0 */
  /*    --> All inputs contain only real numbers */
  /*    --> T(1) <= min(t) <= T(end) */
  /*  */
  /*  All points are considered valid, so extend edge bins: */
  /* 'pwPoly5:25' Tbins = [-inf, T(2:(end-1)), inf]; */
  /* Figure out which bins each query is in: */
  /* 'pwPoly5:28' [~, idx] = histc(t,Tbins); */
  low_i = 0;
  if (!rtIsNaN(unnamed_idx_0 + -1.570796326794897)) {
    if ((unnamed_idx_0 + -1.570796326794897 >= rtMinusInf) && (unnamed_idx_0 +
         -1.570796326794897 < rtInf)) {
      low_i = 1;
      low_ip1 = 2;
      high_i = 7;
      while (high_i > low_ip1) {
        mid_i = (low_i + high_i) >> 1;
        if (unnamed_idx_0 + -1.570796326794897 >= dv0[mid_i - 1]) {
          low_i = mid_i;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }
    }

    if (unnamed_idx_0 + -1.570796326794897 == rtInf) {
      low_i = 7;
    }
  }

  /* 'pwPoly5:28' ~ */
  /*  Call a different sub-function depending on number of outputs: */
  /* 'pwPoly5:31' if nargout == 1 */
  /* 'pwPoly5:32' x = subFun1(T,P,t,idx); */
  /* END MAIN FUNCTION */
  /* %%% Subfunction for a single output (function value) */
  /* 'pwPoly5:46' x = zeros(size(t)); */
  r = 0.0;

  /* 'pwPoly5:47' for i=1:(length(T)-1) */
  for (i = 0; i < 6; i++) {
    /* 'pwPoly5:48' if sum(idx==i)>0 */
    if ((low_i == 1 + i) > 0) {
      /* 'pwPoly5:49' dt = (T(i+1)-T(i)); */
      dt = dv1[1 + i] - dv1[i];

      /* 'pwPoly5:50' tt = (t(idx==i)-T(i))/dt; */
      high_i = 0;
      for (mid_i = 0; mid_i < 1; mid_i++) {
        if (low_i == 1 + i) {
          high_i++;
        }
      }

      tt_size[0] = 1;
      tt_size[1] = high_i;
      for (low_ip1 = 0; low_ip1 < high_i; low_ip1++) {
        tt_data[low_ip1] = ((unnamed_idx_0 + -1.570796326794897) - dv1[i]) / dt;
      }

      /* 'pwPoly5:51' p = P(1,:); */
      /* 'pwPoly5:52' dp = P(2,:)*dt; */
      /* 'pwPoly5:53' ddp = P(3,:)*dt*dt; */
      for (low_ip1 = 0; low_ip1 < 7; low_ip1++) {
        dp[low_ip1] = a[low_ip1] * dt;
        ddp[low_ip1] = b_a[low_ip1] * dt * dt;
      }

      /* 'pwPoly5:54' x(idx==i) = ... */
      /* 'pwPoly5:55'             autoGen_pwPoly5(tt,... */
      /* 'pwPoly5:56'             p(i),dp(i),ddp(i),... */
      /* 'pwPoly5:57'             p(i+1),dp(i+1),ddp(i+1)); */
      /* AUTOGEN_PWPOLY5 */
      /*     [X,DX,DDX] = AUTOGEN_PWPOLY5(T,X0,DX0,DDX0,X1,DX1,DDX1) */
      /*     This function was generated by the Symbolic Math Toolbox version 6.2. */
      /*     15-Apr-2015 11:55:56 */
      /* 'autoGen_pwPoly5:8' t2 = t-1.0; */
      t2_size[0] = 1;
      t2_size[1] = high_i;
      for (low_ip1 = 0; low_ip1 < high_i; low_ip1++) {
        t2_data[low_ip1] = tt_data[low_ip1] - 1.0;
      }

      /* 'autoGen_pwPoly5:9' t3 = t2.^2; */
      power(t2_data, t2_size, t3_data, t3_size);

      /* 'autoGen_pwPoly5:10' t4 = t3.^2; */
      power(t3_data, t3_size, t4_data, t4_size);

      /* 'autoGen_pwPoly5:11' t5 = t.^2; */
      power(tt_data, tt_size, t5_data, t3_size);

      /* 'autoGen_pwPoly5:12' t6 = t5.^2; */
      power(t5_data, t3_size, t6_data, t4_size);

      /* 'autoGen_pwPoly5:13' t7 = dx1.*(1.0./5.0); */
      /* 'autoGen_pwPoly5:14' t8 = t7-x1; */
      t8 = dp[1 + i] * 0.2 - dv2[1 + i];

      /* 'autoGen_pwPoly5:15' t9 = dx0.*(1.0./5.0); */
      /* 'autoGen_pwPoly5:16' t10 = t9+x0; */
      t10 = dp[i] * 0.2 + dv2[i];

      /* 'autoGen_pwPoly5:17' t11 = ddx0.*(1.0./2.0e1); */
      /* 'autoGen_pwPoly5:18' t12 = dx0.*(2.0./5.0); */
      /* 'autoGen_pwPoly5:19' t13 = t11+t12+x0; */
      t13 = (ddp[i] * 0.05 + dp[i] * 0.4) + dv2[i];

      /* 'autoGen_pwPoly5:20' t14 = ddx1.*(1.0./2.0e1); */
      /* 'autoGen_pwPoly5:21' t16 = dx1.*(2.0./5.0); */
      /* 'autoGen_pwPoly5:22' t15 = t14-t16+x1; */
      t15 = (ddp[1 + i] * 0.05 - dp[1 + i] * 0.4) + dv2[1 + i];

      /* 'autoGen_pwPoly5:23' x = t.*t4.*t10.*5.0+t2.*t6.*t8.*5.0-t2.*t4.*x0+t.*t6.*x1+t.*t3.*t5.*t15.*1.0e1-t2.*t3.*t5.*t13.*1.0e1; */
      for (low_ip1 = 0; low_ip1 < high_i; low_ip1++) {
        tt_data[low_ip1] = ((((tt_data[low_ip1] * t4_data[low_ip1] * t10 * 5.0 +
          t2_data[low_ip1] * t6_data[low_ip1] * t8 * 5.0) - t2_data[low_ip1] *
                              t4_data[low_ip1] * dv2[i]) + tt_data[low_ip1] *
                             t6_data[low_ip1] * dv2[1 + i]) + tt_data[low_ip1] *
                            t3_data[low_ip1] * t5_data[low_ip1] * t15 * 10.0) -
          t2_data[low_ip1] * t3_data[low_ip1] * t5_data[low_ip1] * t13 * 10.0;
      }

      /* 'autoGen_pwPoly5:24' if nargout > 1 */
      /* 'autoGen_pwPoly5:27' if nargout > 2 */
      lambda = r;
      low_ip1 = 0;
      for (mid_i = 0; mid_i < 1; mid_i++) {
        if (low_i == 1 + i) {
          lambda = tt_data[low_ip1];
          low_ip1++;
        }
      }

      r = lambda;
    }
  }

  return r;
}

/* End of code generation (footModel.c) */
