/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * motorModel.c
 *
 * Code generation for function 'motorModel'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "simulate.h"
#include "motorModel.h"

/* Function Declarations */
static void RangerJointPhysics(double x, double v, double R, double Vc, double K,
  double G, double c1, double c0, double mu, double Imax, double alpha, double
  Ir, double Cp, double Cd, double xSpring, double kSpring, double *T, double *P,
  double *I);

/* Function Definitions */

/*
 * function [T,P,I] = RangerJointPhysics(x,v,...
 *     R, Vc, K, G, c1, c0, mu,Imax,alpha,...
 *     Ir, Cp, Cd,...
 *     xSpring, kSpring)
 */
static void RangerJointPhysics(double x, double v, double R, double Vc, double K,
  double G, double c1, double c0, double mu, double Imax, double alpha, double
  Ir, double Cp, double Cd, double xSpring, double kSpring, double *T, double *P,
  double *I)
{
  double unnamed_idx_0;
  int i;
  double dv3[1];
  int trueCount;
  double signV;
  double d0;

  /*  */
  /*  */
  /*  This function takes the joint commands and does it's best to match the */
  /*  physics that the robot joint will actually do. */
  /*  */
  /*  INPUTS: */
  /*    x = joint angle */
  /*    v = joint velocity */
  /*    R = 1.3 ohms (terminal resistance) */
  /*    Vc = 0.7 volts (contact voltage drop) */
  /*    K = 0.018 Nm/A (torque constant) */
  /*    G = {66 -> hip, 34 -> ankle} gearbox ratio  */
  /*    c1 = 0 Nms/rad (viscous friction) */
  /*    c0 = 0.01 Nm (constant friction) */
  /*    mu = 0.1 (current-depentent constant friction) */
  /*    Jm = 1.6e-6 kg m^2 (motor inertia) */
  /*    Imax = maximum allowable current command */
  /*    {Ir, Cp, Cd} low-level motor control parameters: */
  /*        I = Ir - Cp*x - Cd*v */
  /*    xSpring = rest angle of the spring */
  /*    kSpring = linear spring constant */
  /*    alpha = smoothing parameter */
  /*  */
  /*  OUTPUTS: */
  /*    T = net torque produced at the joint */
  /*    P = electrical power consumed by the motor */
  /*  */
  /* %% Run the low-level motor controller (essientially hardware level) */
  /* 'motorModel:117' I = Ir - Cp.*x - Cd.*v; */
  *I = (Ir - Cp * x) - Cd * v;

  /* %% Saturate the current if necessary */
  /* 'motorModel:120' I(I>Imax) = Imax; */
  unnamed_idx_0 = *I;
  for (i = 0; i < 1; i++) {
    if (*I > Imax) {
      unnamed_idx_0 = Imax;
    }
  }

  /* 'motorModel:121' I(I<-Imax) = -Imax; */
  dv3[0] = unnamed_idx_0;
  trueCount = 0;
  for (i = 0; i < 1; i++) {
    if (unnamed_idx_0 < -Imax) {
      trueCount++;
    }
  }

  for (i = 0; i < trueCount; i++) {
    dv3[0] = -Imax;
  }

  *I = dv3[0];

  /* %% Compute the net torque */
  /* 'motorModel:124' signV = tanh(v/alpha); */
  signV = tanh(v / alpha);

  /* 'motorModel:125' Tf = -c1*v - c0*signV - mu*G*K*abs(I).*signV; */
  /* Frictional terms */
  /* 'motorModel:126' Te = G*K*I; */
  /*  Electrical terms */
  /* 'motorModel:127' Ts = -kSpring*(x-xSpring); */
  /*  Mechanical spring */
  /* 'motorModel:128' T = Te + Tf + Ts; */
  *T = (G * K * dv3[0] + ((-c1 * v - c0 * signV) - mu * G * K * fabs(dv3[0]) *
         signV)) + -kSpring * (x - xSpring);

  /* %% Compute the electrical power */
  /* 'motorModel:131' P = (I*R + Vc*sign(I)).*I + G*K*v.*I; */
  if (dv3[0] < 0.0) {
    d0 = -1.0;
  } else if (dv3[0] > 0.0) {
    d0 = 1.0;
  } else if (dv3[0] == 0.0) {
    d0 = 0.0;
  } else {
    d0 = dv3[0];
  }

  *P = (dv3[0] * R + Vc * d0) * dv3[0] + G * K * v * dv3[0];
}

/*
 * function [u, power, current] = motorModel(x, iRef, cp, cd, model_motor)
 */
void motorModel(const double x[12], const double iRef[3], const double cp[3],
                const double cd[3], double model_motor_ank_R, double
                model_motor_ank_Vc, double model_motor_ank_K, double
                model_motor_ank_G, double model_motor_ank_c1, double
                model_motor_ank_c0, double model_motor_ank_mu, double
                model_motor_ank_Imax, double model_motor_ank_alpha, double
                model_motor_ank_xSpring, double model_motor_ank_kSpring, const
                struct2_T *model_motor_hip, double model_motor_Phi, double u[3],
                double b_power[3], double current[3])
{
  /*  [u, power, current] = motorModel(x, iRef, cp, cd, model_motor) */
  /*  */
  /*  Implements the low-level motor controller and motor physics model */
  /*  */
  /* 'motorModel:7' angAbs = x(3:6); */
  /* 'motorModel:8' rateAbs = x(9:12); */
  /* 'motorModel:9' [angRel, rateRel] = getAngRel(angAbs,rateAbs, model_motor.Phi); */
  /*  [angRel, rateRel] = getAngRel(angAbs, rateAbs, Phi) */
  /*  */
  /*  This function converts from absolute coordinates to relative coordinates */
  /*  */
  /*  angAbs = [4,n] */
  /*    (1) = phi0 = outer foot absolute angle */
  /*    (2) = phi1 = inner foot absolute angle */
  /*    (3) = th0 = outer foot absolute angle */
  /*    (4) = th1 = inner foot absolute angle */
  /*  */
  /*  angRel = [4,n] */
  /*    (1) = qr = -absolute angle of outer leg (from IMU) */
  /*    (2) = qh = hip joint angle */
  /*    (3) = q0 = outer ankle joint angle */
  /*    (4) = q1 = inner ankle joint angle */
  /*  */
  /*  Phi = ankle orientation constant (~ 1.8 radians) */
  /*  */
  /* 'getAngRel:21' phi0 = angAbs(1,:); */
  /* 'getAngRel:22' phi1 = angAbs(2,:); */
  /* 'getAngRel:23' th0 = angAbs(3,:); */
  /* 'getAngRel:24' th1 = angAbs(4,:); */
  /* 'getAngRel:26' qr = -th0; */
  /* 'getAngRel:27' qh = th1-th0; */
  /* 'getAngRel:28' q0 = Phi - phi0 + th0; */
  /* 'getAngRel:29' q1 = Phi - phi1 + th1; */
  /* 'getAngRel:31' angRel = [qr;qh;q0;q1]; */
  /* 'getAngRel:33' if nargout == 2 */
  /* 'getAngRel:34' dphi0 = rateAbs(1,:); */
  /* 'getAngRel:35' dphi1 = rateAbs(2,:); */
  /* 'getAngRel:36' dth0 = rateAbs(3,:); */
  /* 'getAngRel:37' dth1 = rateAbs(4,:); */
  /* 'getAngRel:39' dqr = -dth0; */
  /* 'getAngRel:40' dqh = dth1-dth0; */
  /* 'getAngRel:41' dq0 = -dphi0 + dth0; */
  /*  NOTE CONSTANT TERM DROPPED */
  /* 'getAngRel:42' dq1 = -dphi1 + dth1; */
  /* 'getAngRel:44' rateRel = [dqr;dqh;dq0;dq1]; */
  /* 'motorModel:11' qh = angRel(2); */
  /* 'motorModel:12' q0 = angRel(3); */
  /* 'motorModel:13' q1 = angRel(4); */
  /* 'motorModel:15' dqh = rateRel(2); */
  /* 'motorModel:16' dq0 = rateRel(3); */
  /* 'motorModel:17' dq1 = rateRel(4); */
  /* 'motorModel:19' u = zeros(3,1); */
  /* Torque to be exerted on each joint */
  /* 'motorModel:20' power = zeros(3,1); */
  /* Electrical power used by each joint */
  /* 'motorModel:21' current = zeros(3,1); */
  /* current sent to each motor */
  /* %%% Outer Ankle Motor: */
  /* 'motorModel:25' x = q0; */
  /* 'motorModel:26' v = dq0; */
  /* 'motorModel:27' R = model_motor.ank.R; */
  /* 'motorModel:28' Vc = model_motor.ank.Vc; */
  /* 'motorModel:29' K = model_motor.ank.K; */
  /* 'motorModel:30' G = model_motor.ank.G; */
  /* 'motorModel:31' c1 = model_motor.ank.c1; */
  /* 'motorModel:32' c0 = model_motor.ank.c0; */
  /* 'motorModel:33' mu = model_motor.ank.mu; */
  /* 'motorModel:34' Imax = model_motor.ank.Imax; */
  /* 'motorModel:35' alpha = model_motor.ank.alpha; */
  /* 'motorModel:36' xSpring = model_motor.ank.xSpring; */
  /* 'motorModel:37' kSpring = model_motor.ank.kSpring; */
  /* 'motorModel:38' [u(1),power(1),current(1)] = RangerJointPhysics(x,v,... */
  /* 'motorModel:39'     R, Vc, K, G, c1, c0, mu,Imax,alpha,... */
  /* 'motorModel:40'     iRef(1), cp(1), cd(1),... */
  /* 'motorModel:41'     xSpring, kSpring); */
  RangerJointPhysics((model_motor_Phi - x[2]) + x[4], -x[8] + x[10],
                     model_motor_ank_R, model_motor_ank_Vc, model_motor_ank_K,
                     model_motor_ank_G, model_motor_ank_c1, model_motor_ank_c0,
                     model_motor_ank_mu, model_motor_ank_Imax,
                     model_motor_ank_alpha, iRef[0], cp[0], cd[0],
                     model_motor_ank_xSpring, model_motor_ank_kSpring, &u[0],
                     &b_power[0], &current[0]);

  /* %%% Inner Ankle Motor: */
  /* 'motorModel:44' x = q1; */
  /* 'motorModel:45' v = dq1; */
  /*  R = motor_model.ank.R; */
  /*  Vc = motor_model.ank.Vc; */
  /*  K = motor_model.ank.K; */
  /*  G = motor_model.ank.G; */
  /*  c1 = motor_model.ank.c1; */
  /*  c0 = motor_model.ank.c0; */
  /*  mu = motor_model.ank.mu; */
  /*  Imax = motor_model.ank.Imax; */
  /*  alpha = motor_model.ank.alpha; */
  /*  xSpring = motor_model.ank.xSpring; */
  /*  kSpring = motor_model.ank.kSpring; */
  /* 'motorModel:57' [u(2),power(2),current(2)] = RangerJointPhysics(x,v,... */
  /* 'motorModel:58'     R, Vc, K, G, c1, c0, mu,Imax,alpha,... */
  /* 'motorModel:59'     iRef(2), cp(2), cd(2),... */
  /* 'motorModel:60'     xSpring, kSpring); */
  RangerJointPhysics((model_motor_Phi - x[3]) + x[5], -x[9] + x[11],
                     model_motor_ank_R, model_motor_ank_Vc, model_motor_ank_K,
                     model_motor_ank_G, model_motor_ank_c1, model_motor_ank_c0,
                     model_motor_ank_mu, model_motor_ank_Imax,
                     model_motor_ank_alpha, iRef[1], cp[1], cd[1],
                     model_motor_ank_xSpring, model_motor_ank_kSpring, &u[1],
                     &b_power[1], &current[1]);

  /* %%% Hip Motor: */
  /* 'motorModel:63' x = qh; */
  /* 'motorModel:64' v = dqh; */
  /* 'motorModel:65' R = model_motor.hip.R; */
  /* 'motorModel:66' Vc = model_motor.hip.Vc; */
  /* 'motorModel:67' K = model_motor.hip.K; */
  /* 'motorModel:68' G = model_motor.hip.G; */
  /* 'motorModel:69' c1 = model_motor.hip.c1; */
  /* 'motorModel:70' c0 = model_motor.hip.c0; */
  /* 'motorModel:71' mu = model_motor.hip.mu; */
  /* 'motorModel:72' Imax = model_motor.hip.Imax; */
  /* 'motorModel:73' alpha = model_motor.hip.alpha; */
  /* 'motorModel:74' xSpring = model_motor.hip.xSpring; */
  /* 'motorModel:75' kSpring = model_motor.hip.kSpring; */
  /* 'motorModel:76' [u(3),power(3),current(3)] = RangerJointPhysics(x,v,... */
  /* 'motorModel:77'     R, Vc, K, G, c1, c0, mu,Imax,alpha,... */
  /* 'motorModel:78'     iRef(3), cp(3), cd(3),... */
  /* 'motorModel:79'     xSpring, kSpring); */
  RangerJointPhysics(x[5] - x[4], x[11] - x[10], model_motor_hip->R,
                     model_motor_hip->Vc, model_motor_hip->K, model_motor_hip->G,
                     model_motor_hip->c1, model_motor_hip->c0,
                     model_motor_hip->mu, model_motor_hip->Imax,
                     model_motor_hip->alpha, iRef[2], cp[2], cd[2],
                     model_motor_hip->xSpring, model_motor_hip->kSpring, &u[2],
                     &b_power[2], &current[2]);
}

/* End of code generation (motorModel.c) */
