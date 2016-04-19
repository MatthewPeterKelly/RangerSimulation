/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * getContactInfo.c
 *
 * Code generation for function 'getContactInfo'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "simulate.h"
#include "getContactInfo.h"
#include "footModel.h"

/* Function Definitions */

/*
 * function [qc, rc] = getContactInfo(pAnkle,qf,param)
 */
void getContactInfo(const double pAnkle[2], double qf, const double
                    param_ground[6], double *qc, double *rc)
{
  double a;
  double b;
  double v;
  double w;
  double thF;
  double d;
  double e;
  double qSoln;
  double x;
  double cx;
  double fx;
  double unusedU0;
  double fv;
  double fw;
  double xm;
  double tol1;
  double tol2;
  int iter;
  int exitg1;
  int gs;
  double r;
  double q;
  double p;
  double b_x;
  double b_d;
  double c_x;
  double fu;

  /*  */
  /*  This function computes the polar coordinates of the contact point, in the */
  /*  frame of the foot. */
  /*  */
  /*  INPUTS: */
  /*    p = position of the ankle joint */
  /*    qf = absolute angle of the foot */
  /*    param = parameter struct */
  /*  */
  /*  OUTPUTS: */
  /*    qc = contact angle, in the frame of the foot */
  /*    rc = radius from ankle joint to contact point, along qc */
  /*  */
  /*  HARD CODED PARAMETERS: */
  /* 'getContactInfo:17' maxIter = 16; */
  /* 'getContactInfo:18' tol = 1e-6; */
  /*  Run optimization to find best point: */
  /* 'getContactInfo:21' [qSoln, ~, rc] = findContactPoint(-pi,pi,maxIter,tol,pAnkle,qf,param.ground); */
  /*  [x, fx, radius] = findContactPoint(thLow,thUpp,maxIter,tol,pAnkle,footRadius,param) */
  /*  */
  /*  Solves a a scalar non-linear optimization problem, using Matlab's fminbnd */
  /*  algorithm, without the added overheads. Designed to be used inside of */
  /*  simulations that are being compiled to mex. */
  /*  */
  /*  MEX does not support function handles, so this optimization routine has */
  /*  the function name hard coded into it as OBJECTIVE_FUNCTION */
  /*  */
  /*  INPUTS: */
  /*    OBJECTIVE_FUNCTION - function handle y = f(x) */
  /*    thLow - lower search bound */
  /*    thUpp - upper search bound */
  /*    maxIter - maximum allowed iterations */
  /*  */
  /*  OUTPUTS: */
  /*    x - location of the minimum */
  /*    yMin - function value at minimum */
  /*    exitFlag - 0=maxIter, 1=success */
  /*  */
  /*  Compute the start point */
  /* 'findContactPoint:24' seps = sqrt(eps); */
  /* 'findContactPoint:25' c = 0.5*(3.0 - sqrt(5.0)); */
  /* 'findContactPoint:26' a = thLow; */
  a = -3.1415926535897931;

  /* 'findContactPoint:26' b = thUpp; */
  b = 3.1415926535897931;

  /* 'findContactPoint:27' v = a + c*(b-a); */
  v = -0.74162942386114006;

  /* 'findContactPoint:28' w = v; */
  w = -0.74162942386114006;

  /* 'findContactPoint:28' thF = v; */
  thF = -0.74162942386114006;

  /* 'findContactPoint:29' d = 0.0; */
  d = 0.0;

  /* 'findContactPoint:29' e = 0.0; */
  e = 0.0;

  /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~% */
  /*                CALL TO THE HARD-CODED OBJECTIVE FUNCTION                 % */
  /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~% */
  /* 'findContactPoint:34' th = thF; */
  qSoln = -0.74162942386114006;

  /* 'findContactPoint:34' [fx, radius] = groundClearance(th,pAnkle,qf,param); */
  /*  */
  /*  This function is used inside of collision detection. It helps determine */
  /*  the point on the foot that is penetrating the ground the most. */
  /*  */
  /*  INPUTS: */
  /*     th = absolute angle of contact point on foot */
  /*     p = [x;y] = location of the ankle joint */
  /*     q = ansolute angle of the foot */
  /*  */
  /*  OUTPUTS: */
  /*     h = penetration between the chosen contact point on the foot and the */
  /*     ground model. */
  /*  */
  /*  Find the radius of the foot in foot reference frame */
  /* 'groundClearance:17' r = footModel(th - q); */
  x = footModel(-0.74162942386114006 - qf);

  /* 'groundClearance:19' [cy, cx] = pol2cart(th,r); */
  /* 'groundClearance:19' cy = -cy; */
  /* Compute the height of the contact point */
  /* 'groundClearance:20' cx = cx + p(1); */
  cx = x * -0.67549029426152374 + pAnkle[0];

  /* 'groundClearance:20' cy = cy + p(2); */
  /* 'groundClearance:22' gy = groundModel(cx,param); */
  /*  [y,n] = groundModel(x,p) */
  /*  */
  /*  This function returns the height (y) and normal vector (n) of the ground */
  /*  model at the horizontal position x. */
  /*  */
  /*  INPUTS: */
  /*    x = scalar horizontal position */
  /*    p = [1, 6] parameter vector */
  /*  */
  /*  OUTPUTS: */
  /*    y = scalar ground height */
  /*    n = [2, 1] ground normal vector */
  /*  */
  /*  NOTES: */
  /*    y = p1 + p2*x + p3*sin(2*pi*(p4 *x + p5)) + p6*x.^2; */
  /*  */
  /*      % Linear */
  /*   %Sine */
  /* 'groundModel:19' y = p(1) + p(2)*x + ...     % Linear */
  /* 'groundModel:20'     p(3)*sin(2*pi*(p(4)*x+p(5))) ...  %Sine */
  /* 'groundModel:21'     + p(6)*x.*x; */
  /*  Quadratic */
  /* 'groundModel:22' if nargout > 1 */
  /* Compute the hight of the ground here */
  /* 'groundClearance:24' h = cy-gy; */
  fx = (-(x * 0.73736887807831974) + pAnkle[1]) - (((param_ground[0] +
    param_ground[1] * cx) + param_ground[2] * sin(6.2831853071795862 *
    (param_ground[3] * cx + param_ground[4]))) + param_ground[5] * cx * cx);

  /* Return the penetration: */
  unusedU0 = fx;

  /* 'findContactPoint:36' fv = fx; */
  fv = fx;

  /* 'findContactPoint:36' fw = fx; */
  fw = fx;

  /* 'findContactPoint:37' xm = 0.5*(a+b); */
  xm = 0.0;

  /* 'findContactPoint:38' tol1 = seps*abs(thF) + tol/3.0; */
  tol1 = 3.4438447292438854E-7;

  /* 'findContactPoint:39' tol2 = 2.0*tol1; */
  tol2 = 6.8876894584877707E-7;

  /*  Main loop */
  /* 'findContactPoint:42' iter = 0; */
  iter = 0;

  /* 'findContactPoint:43' while ( abs(thF-xm) > (tol2 - 0.5*(b-a)) ) */
  do {
    exitg1 = 0;
    if (fabs(thF - xm) > tol2 - 0.5 * (b - a)) {
      /* 'findContactPoint:44' gs = 1; */
      gs = 1;

      /*  Is a parabolic fit possible */
      /* 'findContactPoint:46' if abs(e) > tol1 */
      if (fabs(e) > tol1) {
        /*  Yes, so fit parabola */
        /* 'findContactPoint:48' gs = 0; */
        gs = 0;

        /* 'findContactPoint:49' r = (thF-w)*(fx-fv); */
        r = (thF - w) * (unusedU0 - fv);

        /* 'findContactPoint:50' q = (thF-v)*(fx-fw); */
        q = (thF - v) * (unusedU0 - fw);

        /* 'findContactPoint:51' p = (thF-v)*q-(thF-w)*r; */
        p = (thF - v) * q - (thF - w) * r;

        /* 'findContactPoint:52' q = 2.0*(q-r); */
        q = 2.0 * (q - r);

        /* 'findContactPoint:53' if q > 0.0 */
        if (q > 0.0) {
          /* 'findContactPoint:53' p = -p; */
          p = -p;
        }

        /* 'findContactPoint:54' q = abs(q); */
        q = fabs(q);

        /* 'findContactPoint:55' r = e; */
        r = e;

        /* 'findContactPoint:55' e = d; */
        e = d;

        /*  Is the parabola acceptable */
        /* 'findContactPoint:58' if ( (abs(p)<abs(0.5*q*r)) && (p>q*(a-thF)) && (p<q*(b-thF)) ) */
        if ((fabs(p) < fabs(0.5 * q * r)) && (p > q * (a - thF)) && (p < q * (b
              - thF))) {
          /*  Yes, parabolic interpolation step */
          /* 'findContactPoint:61' d = p/q; */
          d = p / q;

          /* 'findContactPoint:62' th = thF+d; */
          qSoln = thF + d;

          /*  f must not be evaluated too close to ax or bx */
          /* 'findContactPoint:65' if ((th-a) < tol2) || ((b-th) < tol2) */
          if ((qSoln - a < tol2) || (b - qSoln < tol2)) {
            /* 'findContactPoint:66' si = sign(xm-thF) + ((xm-thF) == 0); */
            x = xm - thF;

            /* 'findContactPoint:67' d = tol1*si; */
            if (x < 0.0) {
              b_x = -1.0;
            } else if (x > 0.0) {
              b_x = 1.0;
            } else if (x == 0.0) {
              b_x = 0.0;
            } else {
              b_x = x;
            }

            d = tol1 * (b_x + (double)(xm - thF == 0.0));
          }
        } else {
          /* 'findContactPoint:69' else */
          /*  Not acceptable, must do a golden section step */
          /* 'findContactPoint:71' gs=1; */
          gs = 1;
        }
      }

      /* 'findContactPoint:74' if gs */
      if (gs != 0) {
        /*  A golden-section step is required */
        /* 'findContactPoint:76' if thF >= xm */
        if (thF >= xm) {
          /* 'findContactPoint:76' e = a-thF; */
          e = a - thF;
        } else {
          /* 'findContactPoint:76' else */
          /* 'findContactPoint:76' e = b-thF; */
          e = b - thF;
        }

        /* 'findContactPoint:77' d = c*e; */
        d = 0.3819660112501051 * e;
      }

      /*  The function must not be evaluated too close to thF */
      /* 'findContactPoint:81' si = sign(d) + (d == 0); */
      /* 'findContactPoint:82' th = thF + si * max( abs(d), tol1 ); */
      x = fabs(d);
      if (d < 0.0) {
        b_d = -1.0;
      } else if (d > 0.0) {
        b_d = 1.0;
      } else if (d == 0.0) {
        b_d = 0.0;
      } else {
        b_d = d;
      }

      if ((x >= tol1) || rtIsNaN(tol1)) {
        c_x = x;
      } else {
        c_x = tol1;
      }

      qSoln = thF + (b_d + (double)(d == 0.0)) * c_x;

      /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~% */
      /*            CALL TO THE HARD-CODED OBJECTIVE FUNCTION                 % */
      /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~% */
      /* 'findContactPoint:87' [fu, radius] = groundClearance(th,pAnkle,qf,param); */
      /*  */
      /*  This function is used inside of collision detection. It helps determine */
      /*  the point on the foot that is penetrating the ground the most. */
      /*  */
      /*  INPUTS: */
      /*     th = absolute angle of contact point on foot */
      /*     p = [x;y] = location of the ankle joint */
      /*     q = ansolute angle of the foot */
      /*  */
      /*  OUTPUTS: */
      /*     h = penetration between the chosen contact point on the foot and the */
      /*     ground model. */
      /*  */
      /*  Find the radius of the foot in foot reference frame */
      /* 'groundClearance:17' r = footModel(th - q); */
      x = footModel(qSoln - qf);

      /* 'groundClearance:19' [cy, cx] = pol2cart(th,r); */
      /* 'groundClearance:19' cy = -cy; */
      /* Compute the height of the contact point */
      /* 'groundClearance:20' cx = cx + p(1); */
      cx = x * sin(qSoln) + pAnkle[0];

      /* 'groundClearance:20' cy = cy + p(2); */
      /* 'groundClearance:22' gy = groundModel(cx,param); */
      /*  [y,n] = groundModel(x,p) */
      /*  */
      /*  This function returns the height (y) and normal vector (n) of the ground */
      /*  model at the horizontal position x. */
      /*  */
      /*  INPUTS: */
      /*    x = scalar horizontal position */
      /*    p = [1, 6] parameter vector */
      /*  */
      /*  OUTPUTS: */
      /*    y = scalar ground height */
      /*    n = [2, 1] ground normal vector */
      /*  */
      /*  NOTES: */
      /*    y = p1 + p2*x + p3*sin(2*pi*(p4 *x + p5)) + p6*x.^2; */
      /*  */
      /*      % Linear */
      /*   %Sine */
      /* 'groundModel:19' y = p(1) + p(2)*x + ...     % Linear */
      /* 'groundModel:20'     p(3)*sin(2*pi*(p(4)*x+p(5))) ...  %Sine */
      /* 'groundModel:21'     + p(6)*x.*x; */
      /*  Quadratic */
      /* 'groundModel:22' if nargout > 1 */
      /* Compute the hight of the ground here */
      /* 'groundClearance:24' h = cy-gy; */
      fu = (-(x * cos(qSoln)) + pAnkle[1]) - (((param_ground[0] + param_ground[1]
        * cx) + param_ground[2] * sin(6.2831853071795862 * (param_ground[3] * cx
        + param_ground[4]))) + param_ground[5] * cx * cx);

      /* Return the penetration: */
      /* 'findContactPoint:89' iter = iter + 1; */
      iter++;

      /*  Update a, b, v, w, x, xm, tol1, tol2 */
      /* 'findContactPoint:92' if fu <= fx */
      if (fu <= unusedU0) {
        /* 'findContactPoint:93' if th >= thF */
        if (qSoln >= thF) {
          /* 'findContactPoint:93' a = thF; */
          a = thF;
        } else {
          /* 'findContactPoint:93' else */
          /* 'findContactPoint:93' b = thF; */
          b = thF;
        }

        /* 'findContactPoint:94' v = w; */
        v = w;

        /* 'findContactPoint:94' fv = fw; */
        fv = fw;

        /* 'findContactPoint:95' w = thF; */
        w = thF;

        /* 'findContactPoint:95' fw = fx; */
        fw = unusedU0;

        /* 'findContactPoint:96' thF = th; */
        thF = qSoln;

        /* 'findContactPoint:96' fx = fu; */
        unusedU0 = fu;
      } else {
        /* 'findContactPoint:97' else */
        /*  fu > fx */
        /* 'findContactPoint:98' if th < thF */
        if (qSoln < thF) {
          /* 'findContactPoint:98' a = th; */
          a = qSoln;
        } else {
          /* 'findContactPoint:98' else */
          /* 'findContactPoint:98' b = th; */
          b = qSoln;
        }

        /* 'findContactPoint:99' if ( (fu <= fw) || (w == thF) ) */
        if ((fu <= fw) || (w == thF)) {
          /* 'findContactPoint:100' v = w; */
          v = w;

          /* 'findContactPoint:100' fv = fw; */
          fv = fw;

          /* 'findContactPoint:101' w = th; */
          w = qSoln;

          /* 'findContactPoint:101' fw = fu; */
          fw = fu;
        } else {
          if ((fu <= fv) || (v == thF) || (v == w)) {
            /* 'findContactPoint:102' elseif ( (fu <= fv) || (v == thF) || (v == w) ) */
            /* 'findContactPoint:103' v = th; */
            v = qSoln;

            /* 'findContactPoint:103' fv = fu; */
            fv = fu;
          }
        }
      }

      /* 'findContactPoint:106' xm = 0.5*(a+b); */
      xm = 0.5 * (a + b);

      /* 'findContactPoint:107' tol1 = seps*abs(thF) + tol/3.0; */
      tol1 = 1.4901161193847656E-8 * fabs(thF) + 3.333333333333333E-7;

      /* 'findContactPoint:107' tol2 = 2.0*tol1; */
      tol2 = 2.0 * tol1;

      /* 'findContactPoint:109' if iter >= maxIter */
      if (iter >= 16) {
        /*  Failed to converge - return quietly. "Good enough" is fine here. */
        exitg1 = 1;
      }
    } else {
      /*  while */
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  *rc = x;

  /* 'getContactInfo:21' ~ */
  /* 'getContactInfo:23' qc = qSoln-qf; */
  *qc = qSoln - qf;

  /* Contact angle in foot frame */
}

/* End of code generation (getContactInfo.c) */
