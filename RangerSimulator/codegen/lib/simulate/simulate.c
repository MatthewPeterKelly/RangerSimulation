/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * simulate.c
 *
 * Code generation for function 'simulate'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "simulate.h"
#include "dynamics_ds.h"
#include "motorModel.h"
#include "autoGen_kinematicsContact.h"
#include "mldivide.h"
#include "dynamics_fl.h"
#include "getContactInfo.h"

/* Function Declarations */
static void simulate_fl(const double z[6], const double dz[6], const double qc[2],
  const double rc[2], const double ir[3], const double cp[3], const double cd[3],
  double dyn_g, double dyn_l, double dyn_c, double dyn_m, double dyn_I, double
  dyn_Ifoot, const double dyn_ground[6], double dyn_dt, const struct1_T *motor,
  const double dist[2], double stateNext[12], boolean_T *valid, double u[3],
  double b_power[3], double current[3]);
static void simulate_s0(const double z[6], const double dz[6], const double qc[2],
  const double rc[2], const double ddc0[2], const double n0[2], const double ir
  [3], const double cp[3], const double cd[3], const struct0_T *dyn, const
  struct1_T *motor, const double dist[2], double stateNext[12], double f[4],
  boolean_T *valid, double *violation, double u[3], double b_power[3], double
  current[3]);

/* Function Definitions */

/*
 * function [stateNext,f,valid,u, power, current] = simulate_fl(z,dz,qc,rc,ir, cp, cd, dyn, motor, dist)
 */
static void simulate_fl(const double z[6], const double dz[6], const double qc[2],
  const double rc[2], const double ir[3], const double cp[3], const double cd[3],
  double dyn_g, double dyn_l, double dyn_c, double dyn_m, double dyn_I, double
  dyn_Ifoot, const double dyn_ground[6], double dyn_dt, const struct1_T *motor,
  const double dist[2], double stateNext[12], boolean_T *valid, double u[3],
  double b_power[3], double current[3])
{
  double b_z[12];
  int i;
  double ddz[6];
  double c_z[6];
  double b_ddz;
  double d_z;
  double c1[2];
  double c0[2];
  boolean_T b1;

  /*  Run motor model: */
  /* 'simulate:243' [u, power, current] = motorModel([z;dz], ir, cp, cd, motor); */
  for (i = 0; i < 6; i++) {
    b_z[i] = z[i];
    b_z[i + 6] = dz[i];
  }

  motorModel(b_z, ir, cp, cd, motor->ank.R, motor->ank.Vc, motor->ank.K,
             motor->ank.G, motor->ank.c1, motor->ank.c0, motor->ank.mu,
             motor->ank.Imax, motor->ank.alpha, motor->ank.xSpring,
             motor->ank.kSpring, &motor->hip, motor->Phi, u, b_power, current);

  /*  Solve the dynamics, assuming flight: */
  /* 'simulate:246' ddz = dynamics_fl(z,dz,u,dist,dyn); */
  dynamics_fl(z, dz, u, dist, dyn_g, dyn_c, dyn_m, dyn_I, dyn_Ifoot, ddz);

  /*  March forward in time using symplectic euler integration */
  /* 'simulate:249' [z, dz] = symplecticEuler(z,dz,ddz,dyn.dt); */
  /*  */
  /*  This function computes the next position and velocity, using symplectic */
  /*  euler integration method. */
  /*  */
  /* 'simulate:234' dzNext = dz + h*ddz; */
  /* 'simulate:235' zNext = z + h*dzNext; */
  /*  Update state for next iteration: */
  /* 'simulate:252' stateNext = [z;dz]; */
  for (i = 0; i < 6; i++) {
    b_ddz = dz[i] + dyn_dt * ddz[i];
    d_z = z[i] + dyn_dt * b_ddz;
    stateNext[i] = d_z;
    stateNext[i + 6] = b_ddz;
    c_z[i] = d_z;
  }

  /* 'simulate:254' [c0,c1] = kinematicsContact(z,dz,qc,rc,dyn); */
  /*  [p0c,p1c,dp0c,dp1c] = kinematicsContact(z,dz,qc,rc,dyn) */
  /*  */
  /*  This function computes the kinematics of the contact point on each foot */
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
  /*  */
  /*    qc = [2 x 1] contact point angle relative to foot */
  /*    rc = [2 x 1] contact point radius */
  /*    dyn = parameter struct: */
  /*     .d = eccentricity of the foot */
  /*     .l = leg length (hip joint to foot joint) */
  /*  */
  /*  OUTPUTS: */
  /*    p0c = position of outer foot contact point */
  /*    p1c = position of inner foot contact point */
  /*    dp0c = velocity of outer foot contact point */
  /*    dp1c = velocity of inner foot contact point */
  /*  */
  /*    Angle Conventions: */
  /*        - All angles are in absolute reference frame */
  /*        - zero corresponds to: */
  /*            (legs) = ankle joint directly below hip joint */
  /*            (feet) = ankle joint directly below virtual foot center */
  /*  */
  /* 'kinematicsContact:35' x = z(1); */
  /* 'kinematicsContact:36' y = z(2); */
  /* 'kinematicsContact:37' phi0 = z(3); */
  /* 'kinematicsContact:38' phi1 = z(4); */
  /* 'kinematicsContact:39' th0 = z(5); */
  /* 'kinematicsContact:40' th1 = z(6); */
  /* 'kinematicsContact:42' qc0 = qc(1); */
  /* 'kinematicsContact:43' qc1 = qc(2); */
  /* 'kinematicsContact:44' rc0 = rc(1); */
  /* 'kinematicsContact:45' rc1 = rc(2); */
  /* 'kinematicsContact:47' if nargout > 2 */
  /* 'kinematicsContact:56' if nargout <= 2 */
  /*  Calculate positions onle */
  /* 'kinematicsContact:57' [p0c,p1c] = autoGen_kinematicsContact(... */
  /* 'kinematicsContact:58'         x,y,phi0,phi1,th0,th1,... */
  /* 'kinematicsContact:59'         [],[],[],[],[],[],... */
  /* 'kinematicsContact:60'         qc0,qc1,rc0,rc1,... */
  /* 'kinematicsContact:61'         dyn.l); */
  autoGen_kinematicsContact(c_z[0], c_z[1], c_z[2], c_z[3], c_z[4], c_z[5], qc[0],
    qc[1], rc[0], rc[1], dyn_l, c0, c1);

  /*  Contact forces: */
  /* 'simulate:257' f = zeros(4,1); */
  /*  Compute the ground height at the next contact */
  /* 'simulate:260' g0 = groundModel(c0(1),dyn.ground); */
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
  /* 'simulate:261' g1 = groundModel(c1(1),dyn.ground); */
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
  /*  Check if flight phase is valid: */
  /* 'simulate:264' valid  = c0(2) > g0 && c1(2) > g1; */
  if ((c0[1] > ((dyn_ground[0] + dyn_ground[1] * c0[0]) + dyn_ground[2] * sin
                (6.2831853071795862 * (dyn_ground[3] * c0[0] + dyn_ground[4])))
       + dyn_ground[5] * c0[0] * c0[0]) && (c1[1] > ((dyn_ground[0] +
         dyn_ground[1] * c1[0]) + dyn_ground[2] * sin(6.2831853071795862 *
         (dyn_ground[3] * c1[0] + dyn_ground[4]))) + dyn_ground[5] * c1[0] * c1
       [0])) {
    b1 = true;
  } else {
    b1 = false;
  }

  *valid = b1;
}

/*
 * function [stateNext,f,valid,violation,u,power,current] = simulate_s0(z,dz,qc,rc,ddc0,n0,ir, cp, cd, dyn, motor, dist)
 */
static void simulate_s0(const double z[6], const double dz[6], const double qc[2],
  const double rc[2], const double ddc0[2], const double n0[2], const double ir
  [3], const double cp[3], const double cd[3], const struct0_T *dyn, const
  struct1_T *motor, const double dist[2], double stateNext[12], double f[4],
  boolean_T *valid, double *violation, double u[3], double b_power[3], double
  current[3])
{
  double b_z[12];
  int k;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t8;
  double t9;
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double x[64];
  double M_s0[64];
  double t15;
  double t16;
  double t17;
  double t18;
  double vars[8];
  double c_z[6];
  double b_dz;
  double d_z;
  double c1[2];
  double unusedU0[2];
  double f0n;
  boolean_T b0;

  /*  Run motor model: */
  /* 'simulate:325' [u, power, current] = motorModel([z;dz], ir, cp, cd, motor); */
  for (k = 0; k < 6; k++) {
    b_z[k] = z[k];
    b_z[k + 6] = dz[k];
  }

  motorModel(b_z, ir, cp, cd, motor->ank.R, motor->ank.Vc, motor->ank.K,
             motor->ank.G, motor->ank.c1, motor->ank.c0, motor->ank.mu,
             motor->ank.Imax, motor->ank.alpha, motor->ank.xSpring,
             motor->ank.kSpring, &motor->hip, motor->Phi, u, b_power, current);

  /*  dynamics */
  /* 'simulate:328' [ddz, f0] = dynamics_s0(z,dz,u,dist,qc,rc,ddc0,dyn); */
  /*  [ddz, F0] = dynamics_s0(z,dz,u,f,qc,rc,ddc0,dyn) */
  /*  */
  /*  This function computes the dynamics of the Cornell Ranger, assuming that */
  /*  the acceleration of the contact point on the inner foot is given. */
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
  /*  */
  /*  u = [3, 1] = motor torque vector */
  /*  	(1) = outer ankle joint torque (+leg, -foot) */
  /*  	(2) = inner ankle joint torque (+leg, -foot) */
  /*  	(3) = hip torque (+inner, -outer) */
  /*  */
  /*  f = [2, 1] = disturbance force applied at hip: */
  /*    (1) = fx = horizontal component */
  /*    (2) = fy = vertical component */
  /*  */
  /*    qc = [2 x 1] contact point angle relative to foot */
  /*    rc = [2 x 1] contact point radius */
  /*    ddc0 = [ 2 x 1] foot one contact point acceleration */
  /*  */
  /*  dyn = parameter struct: */
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
  /*  ddz = [6, 1] = full configuration acceleration vector */
  /*    F0 = [2 x 1] contact forces on the inner feet */
  /*    ddc1 = [2 x 1] acceleration of the contact point on the outer feet */
  /*  */
  /*    Angle Conventions: */
  /*        - All angles are in absolute reference frame */
  /*        - zero corresponds to: */
  /*            (legs) = ankle joint directly below hip joint */
  /*            (feet) = ankle joint directly below virtual foot center */
  /*  */
  /*  x = z(1); */
  /*  y = z(2); */
  /* 'dynamics_s0:57' phi0 = z(3); */
  /*  phi1 = z(4); */
  /* 'dynamics_s0:59' th0 = z(5); */
  /* 'dynamics_s0:60' th1 = z(6); */
  /*  dx = dz(1); */
  /*  dy = dz(2); */
  /* 'dynamics_s0:64' dphi0 = dz(3); */
  /*  dphi1 = dz(4); */
  /* 'dynamics_s0:66' dth0 = dz(5); */
  /* 'dynamics_s0:67' dth1 = dz(6); */
  /* 'dynamics_s0:69' u0 = u(1); */
  /* 'dynamics_s0:70' u1 = u(2); */
  /* 'dynamics_s0:71' uHip = u(3); */
  /* 'dynamics_s0:73' fx = f(1); */
  /* 'dynamics_s0:74' fy = f(2); */
  /*  Contact points for each foot */
  /* 'dynamics_s0:77' qc0 = qc(1); */
  /*  qc1 = qc(2); */
  /* 'dynamics_s0:79' rc0 = rc(1); */
  /*  rc1 = rc(2); */
  /*  Acceleration of the virtual center of the foot */
  /* 'dynamics_s0:83' ddp0c_x = ddc0(1); */
  /* 'dynamics_s0:84' ddp0c_y = ddc0(2); */
  /* 'dynamics_s0:86' [M_s0,f_s0] = autoGen_dynamics_s0(... */
  /* 'dynamics_s0:87'     th0,th1,phi0,... */
  /* 'dynamics_s0:88'     dth0,dth1,dphi0,... */
  /* 'dynamics_s0:89'     u0,u1,uHip,... */
  /* 'dynamics_s0:90'     fx, fy,... */
  /* 'dynamics_s0:91'     qc0,rc0,ddp0c_x,ddp0c_y,... */
  /* 'dynamics_s0:92'     dyn.g,dyn.l,dyn.c,dyn.m,dyn.I,dyn.Ifoot,dyn.b); */
  /* AUTOGEN_DYNAMICS_S0 */
  /*     [M_S0,F_S0] = AUTOGEN_DYNAMICS_S0(TH0,TH1,PHI0,DTH0,DTH1,DPHI0,U0,U1,UHIP,FX,FY,QC0,RC0,DDP0C_X,DDP0C_Y,G,L,C,M,I,IFOOT,B) */
  /*     This function was generated by the Symbolic Math Toolbox version 6.3. */
  /*     30-Jan-2016 13:22:39 */
  /* 'autoGen_dynamics_s0:8' t2 = cos(th0); */
  t2 = cos(z[4]);

  /* 'autoGen_dynamics_s0:9' t3 = sin(th0); */
  t3 = sin(z[4]);

  /* 'autoGen_dynamics_s0:10' t4 = phi0+qc0; */
  t4 = z[2] + qc[0];

  /* 'autoGen_dynamics_s0:11' t5 = cos(th1); */
  t5 = cos(z[5]);

  /* 'autoGen_dynamics_s0:12' t6 = sin(th1); */
  t6 = sin(z[5]);

  /* 'autoGen_dynamics_s0:13' t7 = c.^2; */
  /* 'autoGen_dynamics_s0:14' t8 = -I-m.*t7; */
  t8 = -dyn->I - dyn->m * (dyn->c * dyn->c);

  /* 'autoGen_dynamics_s0:15' t9 = cos(t4); */
  t9 = cos(t4);

  /* 'autoGen_dynamics_s0:16' t10 = rc0.*t9; */
  t10 = rc[0] * t9;

  /* 'autoGen_dynamics_s0:17' t11 = sin(t4); */
  t11 = sin(t4);

  /* 'autoGen_dynamics_s0:18' t12 = rc0.*t11; */
  t12 = rc[0] * t11;

  /* 'autoGen_dynamics_s0:19' t13 = l.*t2; */
  t13 = dyn->l * t2;

  /* 'autoGen_dynamics_s0:20' t14 = l.*t3; */
  t14 = dyn->l * t3;

  /* 'autoGen_dynamics_s0:21' M_s0 = reshape([m.*-2.0,0.0,-c.*m.*t2,-c.*m.*t5,0.0,0.0,-1.0,0.0,0.0,m.*-2.0,-c.*m.*t3,-c.*m.*t6,0.0,0.0,0.0,-1.0,0.0,0.0,-Ifoot,0.0,-Ifoot,0.0,-t10,-t12,0.0,0.0,0.0,-Ifoot,0.0,-Ifoot,0.0,0.0,-c.*m.*t2,-c.*m.*t3,t8,0.0,0.0,0.0,-t13,-t14,-c.*m.*t5,-c.*m.*t6,0.0,t8,0.0,0.0,0.0,0.0,1.0,0.0,t10+t13,0.0,t10,0.0,0.0,0.0,0.0,1.0,t12+t14,0.0,t12,0.0,0.0,0.0],[8,8]); */
  x[0] = dyn->m * -2.0;
  x[1] = 0.0;
  x[2] = -dyn->c * dyn->m * t2;
  x[3] = -dyn->c * dyn->m * t5;
  x[4] = 0.0;
  x[5] = 0.0;
  x[6] = -1.0;
  x[7] = 0.0;
  x[8] = 0.0;
  x[9] = dyn->m * -2.0;
  x[10] = -dyn->c * dyn->m * t3;
  x[11] = -dyn->c * dyn->m * t6;
  x[12] = 0.0;
  x[13] = 0.0;
  x[14] = 0.0;
  x[15] = -1.0;
  x[16] = 0.0;
  x[17] = 0.0;
  x[18] = -dyn->Ifoot;
  x[19] = 0.0;
  x[20] = -dyn->Ifoot;
  x[21] = 0.0;
  x[22] = -t10;
  x[23] = -t12;
  x[24] = 0.0;
  x[25] = 0.0;
  x[26] = 0.0;
  x[27] = -dyn->Ifoot;
  x[28] = 0.0;
  x[29] = -dyn->Ifoot;
  x[30] = 0.0;
  x[31] = 0.0;
  x[32] = -dyn->c * dyn->m * t2;
  x[33] = -dyn->c * dyn->m * t3;
  x[34] = t8;
  x[35] = 0.0;
  x[36] = 0.0;
  x[37] = 0.0;
  x[38] = -t13;
  x[39] = -t14;
  x[40] = -dyn->c * dyn->m * t5;
  x[41] = -dyn->c * dyn->m * t6;
  x[42] = 0.0;
  x[43] = t8;
  x[44] = 0.0;
  x[45] = 0.0;
  x[46] = 0.0;
  x[47] = 0.0;
  x[48] = 1.0;
  x[49] = 0.0;
  x[50] = t10 + t13;
  x[51] = 0.0;
  x[52] = t10;
  x[53] = 0.0;
  x[54] = 0.0;
  x[55] = 0.0;
  x[56] = 0.0;
  x[57] = 1.0;
  x[58] = t12 + t14;
  x[59] = 0.0;
  x[60] = t12;
  x[61] = 0.0;
  x[62] = 0.0;
  x[63] = 0.0;
  memcpy(&M_s0[0], &x[0], sizeof(double) << 6);

  /* 'autoGen_dynamics_s0:22' if nargout > 1 */
  /* 'autoGen_dynamics_s0:23' t15 = dth0.^2; */
  t15 = dz[4] * dz[4];

  /* 'autoGen_dynamics_s0:24' t16 = dth1.^2; */
  t16 = dz[5] * dz[5];

  /* 'autoGen_dynamics_s0:25' t17 = b.*dphi0; */
  t17 = dyn->b * dz[2];

  /* 'autoGen_dynamics_s0:26' t18 = dphi0.^2; */
  t18 = dz[2] * dz[2];

  /* 'autoGen_dynamics_s0:27' f_s0 = [-fx-c.*m.*t3.*t15-c.*m.*t6.*t16;-fy+g.*m.*2.0+c.*m.*t2.*t15+c.*m.*t5.*t16;t17+uHip+c.*g.*m.*t3;-uHip+c.*g.*m.*t6;t17+u0;u1;-ddp0c_x-l.*t3.*t15-rc0.*t11.*t18;-ddp0c_y+l.*t2.*t15+rc0.*t9.*t18]; */
  /* 'dynamics_s0:93' vars = M_s0 \ f_s0; */
  vars[0] = (-dist[0] - dyn->c * dyn->m * t3 * t15) - dyn->c * dyn->m * t6 * t16;
  vars[1] = ((-dist[1] + dyn->g * dyn->m * 2.0) + dyn->c * dyn->m * t2 * t15) +
    dyn->c * dyn->m * t5 * t16;
  vars[2] = (t17 + u[2]) + dyn->c * dyn->g * dyn->m * t3;
  vars[3] = -u[2] + dyn->c * dyn->g * dyn->m * t6;
  vars[4] = t17 + u[0];
  vars[5] = u[1];
  vars[6] = (-ddc0[0] - dyn->l * t3 * t15) - rc[0] * t11 * t18;
  vars[7] = (-ddc0[1] + dyn->l * t2 * t15) + rc[0] * t9 * t18;
  mldivide(M_s0, vars);

  /* 'dynamics_s0:95' ddz = vars(1:6); */
  /* 'dynamics_s0:96' F0 = vars(7:8); */
  /*  time-step */
  /* 'simulate:330' [z, dz] = symplecticEuler(z,dz,ddz,dyn.dt); */
  /*  */
  /*  This function computes the next position and velocity, using symplectic */
  /*  euler integration method. */
  /*  */
  /* 'simulate:234' dzNext = dz + h*ddz; */
  /* 'simulate:235' zNext = z + h*dzNext; */
  /* 'simulate:332' [~,c1] = kinematicsContact(z,dz,qc,rc,dyn); */
  /*  [p0c,p1c,dp0c,dp1c] = kinematicsContact(z,dz,qc,rc,dyn) */
  /*  */
  /*  This function computes the kinematics of the contact point on each foot */
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
  /*  */
  /*    qc = [2 x 1] contact point angle relative to foot */
  /*    rc = [2 x 1] contact point radius */
  /*    dyn = parameter struct: */
  /*     .d = eccentricity of the foot */
  /*     .l = leg length (hip joint to foot joint) */
  /*  */
  /*  OUTPUTS: */
  /*    p0c = position of outer foot contact point */
  /*    p1c = position of inner foot contact point */
  /*    dp0c = velocity of outer foot contact point */
  /*    dp1c = velocity of inner foot contact point */
  /*  */
  /*    Angle Conventions: */
  /*        - All angles are in absolute reference frame */
  /*        - zero corresponds to: */
  /*            (legs) = ankle joint directly below hip joint */
  /*            (feet) = ankle joint directly below virtual foot center */
  /*  */
  /* 'kinematicsContact:35' x = z(1); */
  /* 'kinematicsContact:36' y = z(2); */
  /* 'kinematicsContact:37' phi0 = z(3); */
  /* 'kinematicsContact:38' phi1 = z(4); */
  /* 'kinematicsContact:39' th0 = z(5); */
  /* 'kinematicsContact:40' th1 = z(6); */
  /* 'kinematicsContact:42' qc0 = qc(1); */
  /* 'kinematicsContact:43' qc1 = qc(2); */
  /* 'kinematicsContact:44' rc0 = rc(1); */
  /* 'kinematicsContact:45' rc1 = rc(2); */
  /* 'kinematicsContact:47' if nargout > 2 */
  /* 'kinematicsContact:56' if nargout <= 2 */
  /*  Calculate positions onle */
  /* 'kinematicsContact:57' [p0c,p1c] = autoGen_kinematicsContact(... */
  /* 'kinematicsContact:58'         x,y,phi0,phi1,th0,th1,... */
  /* 'kinematicsContact:59'         [],[],[],[],[],[],... */
  /* 'kinematicsContact:60'         qc0,qc1,rc0,rc1,... */
  /* 'kinematicsContact:61'         dyn.l); */
  for (k = 0; k < 6; k++) {
    b_dz = dz[k] + dyn->dt * vars[k];
    d_z = z[k] + dyn->dt * b_dz;
    stateNext[k] = d_z;
    stateNext[k + 6] = b_dz;
    c_z[k] = d_z;
  }

  autoGen_kinematicsContact(c_z[0], c_z[1], c_z[2], c_z[3], c_z[4], c_z[5], qc[0],
    qc[1], rc[0], rc[1], dyn->l, unusedU0, c1);

  /* 'simulate:332' ~ */
  /* 'simulate:334' stateNext = [z;dz]; */
  /* 'simulate:335' f = [f0; 0;0]; */
  f[2] = 0.0;
  f[3] = 0.0;

  /*  check assumption: */
  /* 'simulate:338' f0n = dot(f0,n0); */
  f0n = 0.0;
  for (k = 0; k < 2; k++) {
    f[k] = vars[k + 6];
    f0n += vars[k + 6] * n0[k];
  }

  /* 'simulate:339' valid = c1(2) > groundModel(c1(1),dyn.ground) && f0n > 0; */
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
  if ((c1[1] > ((dyn->ground[0] + dyn->ground[1] * c1[0]) + dyn->ground[2] * sin
                (6.2831853071795862 * (dyn->ground[3] * c1[0] + dyn->ground[4])))
       + dyn->ground[5] * c1[0] * c1[0]) && (f0n > 0.0)) {
    b0 = true;
  } else {
    b0 = false;
  }

  *valid = b0;

  /*  Check relaxed assumptiona: */
  /* 'simulate:342' violation = -f0n; */
  *violation = -f0n;
}

/*
 * function [stateNext,f,c,k,torque, power, current] = simulate(state, ir, cp, cd, dyn, motor, dist)
 */
void simulate(const double state[12], const double ir[3], const double cp[3],
              const double cd[3], const struct0_T *dyn, const struct1_T *motor,
              const double dist[2], double stateNext[12], double f[4], double c
              [4], boolean_T k[2], double torque[3], double b_power[3], double
              current[3])
{
  double b_state[2];
  double rc0;
  double qc0;
  double c_state[2];
  double rc1;
  double qc1;
  double qc[2];
  double rc[2];
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double c0[2];
  double t8;
  double t9;
  double t10;
  double t11;
  double p1c[2];
  int i;
  double g0;
  double dy;
  double x;
  double n0[2];
  double g1;
  double n1[2];
  double wn;
  double kp;
  double kd;
  double ddc0[2];
  double ddc1[2];
  boolean_T contactOut;
  boolean_T contactInn;
  signed char order[4];
  int b_k;
  static const signed char iv0[4] = { 0, 2, 1, 3 };

  static const signed char iv1[4] = { 1, 2, 0, 3 };

  static const signed char iv2[4] = { 2, 0, 1, 3 };

  static const signed char iv3[4] = { 3, 0, 2, 1 };

  int exitg1;
  boolean_T guard1 = false;
  double S0_current[3];
  double S0_power[3];
  double S0_torque[3];
  boolean_T S0_valid;
  double n[4];
  double S0_stateNext[12];
  static const boolean_T bv0[2] = { true, false };

  double d_state[12];
  double t12;
  double t13;
  double t14;
  double b_x[64];
  double M_s1[64];
  double t15;
  double t16;
  double t17;
  double t18;
  double vars[8];
  double dz[6];
  double z[6];
  double b_dz;
  double f1n;
  boolean_T S1_valid;
  static const boolean_T bv1[2] = { false, true };

  double b_ddc0[4];
  double fn0;
  double fn1;
  boolean_T DS_valid;
  boolean_T FL_valid;

  /*  [stateNext,f,c,k,torque, power, current] = simulate(state, ir, cp, cd, dyn, motor, dist) */
  /*  */
  /*  This function computes a single time-step for the Cornell Ranger, */
  /*  assuming that it is walking on flat ground, the feet roll without */
  /*  slipping, and the coefficient of restitution for the ground is zero. The */
  /*  time-step is computed using symplectic euler integration. The contact */
  /*  constraints are approximately satisfied on each time step, with errors */
  /*  decaying asymtotically over a few time steps. */
  /*  */
  /*  INPUTS: */
  /*  state = [z;dz] state vector */
  /*    z = [6, 1] = full configuration vector */
  /*    dz = [6, 1] = full configuration rate vector */
  /*        (1) = hip horizontal position */
  /*        (2) = hip vertical position */
  /*        (3) = outer foot absolute angle */
  /*        (4) = inner foot absolute angle */
  /*        (5) = outer leg absolute angle */
  /*        (6) = inner leg absolute angle */
  /*  */
  /*  ir = reference current = [3, 1] = motor current set-point */
  /*  cp = proportional gain (current) = [3, 1] = motor controller */
  /*  cd = derivative gain (current) = [3, 1] = motor controller */
  /*  	(1) = outer ankle joint torque (+leg, -foot) */
  /*  	(2) = inner ankle joint torque (+leg, -foot) */
  /*  	(3) = hip torque (+inner, -outer) */
  /*  */
  /*  dyn = parameter struct: */
  /*     .g = acceleration due to gravity */
  /*     .l = leg length (hip joint to foot joint) */
  /*     .d = distance between foot joint and virtual center of foot */
  /*     .r = radius of circular arc on foot */
  /*     .c = distance along leg from hip to CoM */
  /*     .w = distance off leg from main axis to CoM */
  /*     .m = mass of each leg */
  /*     .I = moment of inertia of the leg about its center of mass */
  /*     .Ifoot = moment of inertia of the foot about the foot joint */
  /*     .dt = time step for integration method */
  /*  */
  /*  motor = struct with motor paramters: */
  /*  */
  /*     .ank.R = 1.3; %ohms (terminal resistance) */
  /*     .ank.Vc = 0.7; %volts (contact voltage drop) */
  /*     .ank.K = 0.018; %Nm/A (torque constant) */
  /*     .ank.G = 34;%{66 -> hip, 34 -> ankle} gearbox ratio */
  /*     .ank.c1 = 0; %Nms/rad (viscous friction) */
  /*     .ank.c0 = 0.01; %Nm (constant friction) */
  /*     .ank.mu = 0.1; %(current-depentent constant friction) */
  /*     .ank.Imax = 8.0; %maximum allowable motor current (3.1 continuous, 8.0 peak) */
  /*     .ank.alpha = 0.02;  %Smoothing parameter for sign() function */
  /*     .ank.xSpring = 1.662; */
  /*     .ank.kSpring = 0.134; */
  /*  */
  /*     .hip = *.ank  %Most fields are the same, but a few (below) are not: */
  /*     .hip.G = 66; */
  /*     .hip.xSpring = 0.0; */
  /*     .hip.kSpring = 8.045; */
  /*  */
  /*  dist = [2,1] = column vector of disturbances applied at hip */
  /*    (1) = horizontal component */
  /*    (2) = vertical comonent */
  /*  */
  /*  OUTPUTS: */
  /*    state = [12 x 1] state vector at next time step */
  /*  f = [4, 1] = contact force vector */
  /*  f = [f0; f1] = [outer; inner] contact forces */
  /*  	(1) = horizontal component */
  /*  	(2) = vertical component */
  /*  c = [4 x 1] = [c0;c1] = [outer;inner] = [x;y;x;y] contact points */
  /*  k = [2 x 1] boolean vector [outerFootInContact; innerFootInContact]; */
  /*  torque = [u0; u1; uHip];   %Realized torques at each joint */
  /*  power = power used by each joint, same order as torques */
  /*  current = current sent to each joint, same order as torques */
  /*  */
  /*  NOTES: */
  /*  */
  /*    Angle Conventions: */
  /*        - All angles are in absolute reference frame */
  /*        - zero corresponds to: */
  /*            (legs) = ankle joint directly below hip joint */
  /*            (feet) = ankle joint directly below virtual foot center */
  /*  */
  /* 'simulate:85' z = state(1:6); */
  /* 'simulate:86' dz = state(7:12); */
  /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~% */
  /*                            Kinematics                                    % */
  /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~% */
  /*  Determine where the contact points on each foot would be, if they were in */
  /*  contact, and also determine the acceleration of the virtual center of */
  /*  each foot that would be consistent with that contact mode. */
  /*  Figure out the contact angle for each foot: */
  /* 'simulate:96' [p0,p1] = kinematics(z,[],[],dyn); */
  /*  [p0,p1,dp0,dp1,ddp0,ddp1] = kinematics(z,dz,ddz,dyn) */
  /*  */
  /*  This function computes the kinematics of the ankle joints of the robot */
  /*  */
  /*  INPUTS: */
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
  /*  */
  /*  OUTPUTS: */
  /*    p0 = position of outer ankle joint */
  /*    p1 = position of inner ankle joint */
  /*    dp0 = position of outer ankle joint */
  /*    dp1 = velocity of inner ankle joint */
  /*    ddp0 = position of outer ankle joint */
  /*    ddp1 = acceleration inner ankle joint */
  /*  */
  /*    Kinematics: */
  /*        1 = horizontal (x) */
  /*        2 = vertical (y) */
  /*    Angle Conventions: */
  /*        - All angles are in absolute reference frame */
  /*        - zero corresponds to: */
  /*            (legs) = ankle joint directly below hip joint */
  /*            (feet) = ankle joint directly below virtual foot center */
  /*  */
  /* 'kinematics:47' x = z(1); */
  /* 'kinematics:48' y = z(2); */
  /*  phi0 = z(3); */
  /*  phi1 = z(4); */
  /* 'kinematics:51' th0 = z(5); */
  /* 'kinematics:52' th1 = z(6); */
  /* 'kinematics:54' if nargout > 4 */
  /* 'kinematics:64' if nargout > 8 */
  /* 'kinematics:73' if nargout <= 4 */
  /*  Calculate positions onle */
  /* 'kinematics:74' [p0,p1] = autoGen_kinematics(... */
  /* 'kinematics:75'         x,y,th0,th1,... */
  /* 'kinematics:76'         [],[],[],[],... */
  /* 'kinematics:77'         [],[],[],[],... */
  /* 'kinematics:78'         dyn.l); */
  /* AUTOGEN_KINEMATICS */
  /*     [P0,P1,DP0,DP1,DDP0,DDP1] = AUTOGEN_KINEMATICS(X,Y,TH0,TH1,DX,DY,DTH0,DTH1,DDX,DDY,DDTH0,DDTH1,L) */
  /*     This function was generated by the Symbolic Math Toolbox version 6.3. */
  /*     30-Jan-2016 13:22:37 */
  /* 'autoGen_kinematics:8' t2 = cos(th0); */
  /* 'autoGen_kinematics:9' t3 = sin(th0); */
  /* 'autoGen_kinematics:10' p0 = [x+l.*t3;y-l.*t2]; */
  /* 'autoGen_kinematics:11' if nargout > 1 */
  /* 'autoGen_kinematics:12' t4 = cos(th1); */
  /* 'autoGen_kinematics:13' t5 = sin(th1); */
  /* 'autoGen_kinematics:14' p1 = [x+l.*t5;y-l.*t4]; */
  /* 'autoGen_kinematics:16' if nargout > 2 */
  /* 'autoGen_kinematics:19' if nargout > 3 */
  /* 'autoGen_kinematics:22' if nargout > 4 */
  /* 'autoGen_kinematics:26' if nargout > 5 */
  /* 'simulate:97' phi0 = z(3); */
  /* 'simulate:97' phi1 = z(4); */
  /* 'simulate:98' [qc0, rc0] = getContactInfo(p0,phi0,dyn); */
  b_state[0] = state[0] + dyn->l * sin(state[4]);
  b_state[1] = state[1] - dyn->l * cos(state[4]);
  getContactInfo(b_state, state[2], dyn->ground, &qc0, &rc0);

  /* 'simulate:99' [qc1, rc1] = getContactInfo(p1,phi1,dyn); */
  c_state[0] = state[0] + dyn->l * sin(state[5]);
  c_state[1] = state[1] - dyn->l * cos(state[5]);
  getContactInfo(c_state, state[3], dyn->ground, &qc1, &rc1);

  /* 'simulate:100' qc = [qc0;qc1]; */
  qc[0] = qc0;
  qc[1] = qc1;

  /* 'simulate:100' rc = [rc0;rc1]; */
  rc[0] = rc0;
  rc[1] = rc1;

  /*  Determine the kinematics of each contact point */
  /* 'simulate:103' [c0,c1,dc0,dc1] = kinematicsContact(z,dz,qc,rc,dyn); */
  /*  [p0c,p1c,dp0c,dp1c] = kinematicsContact(z,dz,qc,rc,dyn) */
  /*  */
  /*  This function computes the kinematics of the contact point on each foot */
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
  /*  */
  /*    qc = [2 x 1] contact point angle relative to foot */
  /*    rc = [2 x 1] contact point radius */
  /*    dyn = parameter struct: */
  /*     .d = eccentricity of the foot */
  /*     .l = leg length (hip joint to foot joint) */
  /*  */
  /*  OUTPUTS: */
  /*    p0c = position of outer foot contact point */
  /*    p1c = position of inner foot contact point */
  /*    dp0c = velocity of outer foot contact point */
  /*    dp1c = velocity of inner foot contact point */
  /*  */
  /*    Angle Conventions: */
  /*        - All angles are in absolute reference frame */
  /*        - zero corresponds to: */
  /*            (legs) = ankle joint directly below hip joint */
  /*            (feet) = ankle joint directly below virtual foot center */
  /*  */
  /* 'kinematicsContact:35' x = z(1); */
  /* 'kinematicsContact:36' y = z(2); */
  /* 'kinematicsContact:37' phi0 = z(3); */
  /* 'kinematicsContact:38' phi1 = z(4); */
  /* 'kinematicsContact:39' th0 = z(5); */
  /* 'kinematicsContact:40' th1 = z(6); */
  /* 'kinematicsContact:42' qc0 = qc(1); */
  /* 'kinematicsContact:43' qc1 = qc(2); */
  /* 'kinematicsContact:44' rc0 = rc(1); */
  /* 'kinematicsContact:45' rc1 = rc(2); */
  /* 'kinematicsContact:47' if nargout > 2 */
  /* Then need velocities */
  /* 'kinematicsContact:48' dx = dz(1); */
  /* 'kinematicsContact:49' dy = dz(2); */
  /* 'kinematicsContact:50' dphi0 = dz(3); */
  /* 'kinematicsContact:51' dphi1 = dz(4); */
  /* 'kinematicsContact:52' dth0 = dz(5); */
  /* 'kinematicsContact:53' dth1 = dz(6); */
  /* 'kinematicsContact:56' if nargout <= 2 */
  /* 'kinematicsContact:62' elseif nargout > 2 */
  /*  calculate positions and velocities */
  /* 'kinematicsContact:63' [p0c,p1c,dp0c,dp1c] = autoGen_kinematicsContact(... */
  /* 'kinematicsContact:64'         x,y,phi0,phi1,th0,th1,... */
  /* 'kinematicsContact:65'         dx,dy,dphi0,dphi1,dth0,dth1,... */
  /* 'kinematicsContact:66'         qc0,qc1,rc0,rc1,... */
  /* 'kinematicsContact:67'         dyn.l); */
  /* AUTOGEN_KINEMATICSCONTACT */
  /*     [P0C,P1C,DP0C,DP1C] = AUTOGEN_KINEMATICSCONTACT(X,Y,PHI0,PHI1,TH0,TH1,DX,DY,DPHI0,DPHI1,DTH0,DTH1,QC0,QC1,RC0,RC1,L) */
  /*     This function was generated by the Symbolic Math Toolbox version 6.3. */
  /*     30-Jan-2016 13:22:37 */
  /* 'autoGen_kinematicsContact:8' t2 = phi0+qc0; */
  t2 = state[2] + qc0;

  /* 'autoGen_kinematicsContact:9' t3 = phi1+qc1; */
  t3 = state[3] + qc1;

  /* 'autoGen_kinematicsContact:10' t4 = cos(th0); */
  t4 = cos(state[4]);

  /* 'autoGen_kinematicsContact:11' t5 = cos(t2); */
  t5 = cos(t2);

  /* 'autoGen_kinematicsContact:12' t6 = sin(th0); */
  t6 = sin(state[4]);

  /* 'autoGen_kinematicsContact:13' t7 = sin(t2); */
  t7 = sin(t2);

  /* 'autoGen_kinematicsContact:14' p0c = [x+l.*t6+rc0.*t7;y-l.*t4-rc0.*t5]; */
  c0[0] = (state[0] + dyn->l * t6) + rc0 * t7;
  c0[1] = (state[1] - dyn->l * t4) - rc0 * t5;

  /* 'autoGen_kinematicsContact:15' if nargout > 1 */
  /* 'autoGen_kinematicsContact:16' t8 = cos(th1); */
  t8 = cos(state[5]);

  /* 'autoGen_kinematicsContact:17' t9 = cos(t3); */
  t9 = cos(t3);

  /* 'autoGen_kinematicsContact:18' t10 = sin(th1); */
  t10 = sin(state[5]);

  /* 'autoGen_kinematicsContact:19' t11 = sin(t3); */
  t11 = sin(t3);

  /* 'autoGen_kinematicsContact:20' p1c = [x+l.*t10+rc1.*t11;y-l.*t8-rc1.*t9]; */
  p1c[0] = (state[0] + dyn->l * t10) + rc1 * t11;
  p1c[1] = (state[1] - dyn->l * t8) - rc1 * t9;

  /* 'autoGen_kinematicsContact:22' if nargout > 2 */
  /* 'autoGen_kinematicsContact:23' dp0c = [dx+dth0.*l.*t4+dphi0.*rc0.*t5;dy+dth0.*l.*t6+dphi0.*rc0.*t7]; */
  /* 'autoGen_kinematicsContact:25' if nargout > 3 */
  /* 'autoGen_kinematicsContact:26' dp1c = [dx+dth1.*l.*t8+dphi1.*rc1.*t9;dy+dth1.*l.*t10+dphi1.*rc1.*t11]; */
  /* 'simulate:104' c = [c0;c1]; */
  for (i = 0; i < 2; i++) {
    c[i] = c0[i];
    c[i + 2] = p1c[i];
  }

  /* Contact points for each foot at the start of the time step */
  /*  Compute the ground height and normal vector at each contact: */
  /* 'simulate:107' [g0, n0] = groundModel(c0(1),dyn.ground); */
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
  g0 = ((dyn->ground[0] + dyn->ground[1] * c0[0]) + dyn->ground[2] * sin
        (6.2831853071795862 * (dyn->ground[3] * c0[0] + dyn->ground[4]))) +
    dyn->ground[5] * c0[0] * c0[0];

  /*  Quadratic */
  /* 'groundModel:22' if nargout > 1 */
  /* 'groundModel:23' dy = p(2) + ... */
  /* 'groundModel:24'         p(3)*p(4)*2*pi*cos(2*pi*(p(4)*x+p(5))) + ... */
  /* 'groundModel:25'         2*p(6)*x; */
  dy = (dyn->ground[1] + dyn->ground[2] * dyn->ground[3] * 2.0 *
        3.1415926535897931 * cos(6.2831853071795862 * (dyn->ground[3] * c0[0] +
          dyn->ground[4]))) + 2.0 * dyn->ground[5] * c0[0];

  /* 'groundModel:26' n = [-dy;1]/sqrt(1+dy*dy); */
  x = sqrt(1.0 + dy * dy);
  n0[0] = -dy / x;
  n0[1] = 1.0 / x;

  /* 'simulate:108' [g1, n1] = groundModel(c1(1),dyn.ground); */
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
  g1 = ((dyn->ground[0] + dyn->ground[1] * p1c[0]) + dyn->ground[2] * sin
        (6.2831853071795862 * (dyn->ground[3] * p1c[0] + dyn->ground[4]))) +
    dyn->ground[5] * p1c[0] * p1c[0];

  /*  Quadratic */
  /* 'groundModel:22' if nargout > 1 */
  /* 'groundModel:23' dy = p(2) + ... */
  /* 'groundModel:24'         p(3)*p(4)*2*pi*cos(2*pi*(p(4)*x+p(5))) + ... */
  /* 'groundModel:25'         2*p(6)*x; */
  dy = (dyn->ground[1] + dyn->ground[2] * dyn->ground[3] * 2.0 *
        3.1415926535897931 * cos(6.2831853071795862 * (dyn->ground[3] * p1c[0] +
          dyn->ground[4]))) + 2.0 * dyn->ground[5] * p1c[0];

  /* 'groundModel:26' n = [-dy;1]/sqrt(1+dy*dy); */
  x = sqrt(1.0 + dy * dy);
  n1[0] = -dy / x;
  n1[1] = 1.0 / x;

  /* Project contact points to surface */
  /* 'simulate:111' c0Next = [c0(1); g0]; */
  /* 'simulate:112' c1Next = [c1(1); g1]; */
  /*  Constraint solver: */
  /* 'simulate:115' h = dyn.dt; */
  /* 'simulate:116' wn = dyn.cstWn/(2*h); */
  wn = dyn->cstWn / (2.0 * dyn->dt);

  /*  Stiffness of contact solver (feet and ground interaction) */
  /* 'simulate:117' xi = dyn.cstXi; */
  /*  Damping ratio for the contact solver */
  /* 'simulate:118' kp = wn*wn; */
  kp = wn * wn;

  /* 'simulate:119' kd = 2*xi*wn; */
  kd = 2.0 * dyn->cstXi * wn;

  /* 'simulate:120' e0 = c0Next - c0; */
  /* 'simulate:121' de0 = -dc0; */
  /* 'simulate:122' e1 = c1Next - c1; */
  /* 'simulate:123' de1 = -dc1; */
  /* 'simulate:124' ddc0 = kp*e0 + kd*de0; */
  ddc0[0] = kp * (c0[0] - c0[0]) + kd * -((state[6] + state[10] * dyn->l * t4) +
    state[8] * rc0 * t5);
  ddc0[1] = kp * (g0 - c0[1]) + kd * -((state[7] + state[10] * dyn->l * t6) +
    state[8] * rc0 * t7);

  /* 'simulate:125' ddc1 = kp*e1 + kd*de1; */
  ddc1[0] = kp * (p1c[0] - p1c[0]) + kd * -((state[6] + state[11] * dyn->l * t8)
    + state[9] * rc1 * t9);
  ddc1[1] = kp * (g1 - p1c[1]) + kd * -((state[7] + state[11] * dyn->l * t10) +
    state[9] * rc1 * t11);

  /*  Figure out which constraints are in violation: */
  /* 'simulate:128' contactOut = c0(2) < g0; */
  contactOut = (c0[1] < g0);

  /* Contact on outer foot is active */
  /* 'simulate:129' contactInn = c1(2) < g1; */
  contactInn = (p1c[1] < g1);

  /* Contact on inner foot is active */
  /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~% */
  /*                Compute dynamics for each mode                            % */
  /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~% */
  /* Make a guess at which order to attempt contacts: */
  /* 'simulate:137' if contactOut && ~contactInn */
  if (contactOut && (!contactInn)) {
    /* Try single stance outer first */
    /* 'simulate:138' order = [0,2,1,3]; */
    for (b_k = 0; b_k < 4; b_k++) {
      order[b_k] = iv0[b_k];
    }
  } else if ((!contactOut) && contactInn) {
    /* 'simulate:139' elseif ~contactOut && contactInn */
    /* Try single stance inner first */
    /* 'simulate:140' order = [1,2,0,3]; */
    for (b_k = 0; b_k < 4; b_k++) {
      order[b_k] = iv1[b_k];
    }
  } else if (contactOut && contactInn) {
    /* 'simulate:141' elseif contactOut && contactInn */
    /* Try double stance first */
    /* 'simulate:142' order = [2,0,1,3]; */
    for (b_k = 0; b_k < 4; b_k++) {
      order[b_k] = iv2[b_k];
    }
  } else {
    /* 'simulate:143' else */
    /* 'simulate:144' order = [3,0,2,1]; */
    for (b_k = 0; b_k < 4; b_k++) {
      order[b_k] = iv3[b_k];
    }

    /* Try flight first */
  }

  /* 'simulate:147' XX_vln = 0; */
  /*  Contact force violation; */
  /* 'simulate:148' XX_stateNext = zeros(12,1); */
  memset(&stateNext[0], 0, 12U * sizeof(double));

  /* 'simulate:149' XX_f = zeros(4,1); */
  for (i = 0; i < 4; i++) {
    f[i] = 0.0;
  }

  /* 'simulate:150' XX_k = false(2,1); */
  for (i = 0; i < 2; i++) {
    k[i] = false;
  }

  /* 'simulate:151' XX_torque = zeros(3,1); */
  /* 'simulate:152' XX_power = zeros(3,1); */
  /* 'simulate:153' XX_current = zeros(3,1); */
  for (i = 0; i < 3; i++) {
    torque[i] = 0.0;
    b_power[i] = 0.0;
    current[i] = 0.0;
  }

  /* 'simulate:154' for i=order */
  i = 0;
  do {
    exitg1 = 0;
    if (i < 4) {
      /* 'simulate:155' switch i */
      guard1 = false;
      switch (order[i]) {
       case 0:
        /* 'simulate:156' case 0 %single stance outer */
        /* single stance outer */
        /* 'simulate:157' [S0_stateNext, S0_f, S0_valid, S0_vln, S0_torque, S0_power, S0_current]... */
        /* 'simulate:158'                 = simulate_s0(z,dz,qc,rc,ddc0,n1,ir, cp, cd, dyn, motor, dist); */
        simulate_s0(*(double (*)[6])&state[0], *(double (*)[6])&state[6], qc, rc,
                    ddc0, n1, ir, cp, cd, dyn, motor, dist, S0_stateNext, n,
                    &S0_valid, &x, S0_torque, S0_power, S0_current);

        /* 'simulate:159' if S0_valid */
        if (S0_valid) {
          /* 'simulate:160' stateNext = S0_stateNext; */
          memcpy(&stateNext[0], &S0_stateNext[0], 12U * sizeof(double));

          /* 'simulate:160' f = S0_f; */
          for (i = 0; i < 4; i++) {
            f[i] = n[i];
          }

          /* 'simulate:160' k = [true; false]; */
          for (i = 0; i < 2; i++) {
            k[i] = bv0[i];
          }

          /* 'simulate:161' torque = S0_torque; */
          /* 'simulate:161' power = S0_power; */
          /* 'simulate:161' current = S0_current; */
          for (i = 0; i < 3; i++) {
            torque[i] = S0_torque[i];
            b_power[i] = S0_power[i];
            current[i] = S0_current[i];
          }

          exitg1 = 1;
        } else {
          guard1 = true;
        }
        break;

       case 1:
        /* 'simulate:168' case 1 %single stance - inner legs */
        /* single stance - inner legs */
        /* 'simulate:169' [S1_stateNext, S1_f, S1_valid, S1_vln, S1_torque, S1_power, S1_current]... */
        /* 'simulate:170'                 = simulate_s1(z,dz,qc,rc,ddc1,n0,ir, cp, cd, dyn, motor, dist); */
        /*  Run motor model: */
        /* 'simulate:300' [u, power, current] = motorModel([z;dz], ir, cp, cd, motor); */
        for (b_k = 0; b_k < 6; b_k++) {
          d_state[b_k] = state[b_k];
          d_state[b_k + 6] = state[b_k + 6];
        }

        motorModel(d_state, ir, cp, cd, motor->ank.R, motor->ank.Vc,
                   motor->ank.K, motor->ank.G, motor->ank.c1, motor->ank.c0,
                   motor->ank.mu, motor->ank.Imax, motor->ank.alpha,
                   motor->ank.xSpring, motor->ank.kSpring, &motor->hip,
                   motor->Phi, S0_torque, S0_power, S0_current);

        /*  dynamics */
        /* 'simulate:303' [ddz, f1] = dynamics_s1(z,dz,u,dist,qc,rc,ddc1,dyn); */
        /*  [ddz, F1] = dynamics_s1(z,dz,u,f,qc,rc,ddc1,dyn) */
        /*  */
        /*  This function computes the dynamics of the Cornell Ranger, assuming that */
        /*  the acceleration of the contact point on the inner foot is given. */
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
        /*  */
        /*  u = [3, 1] = motor torque vector */
        /*  	(1) = outer ankle joint torque (+leg, -foot) */
        /*  	(2) = inner ankle joint torque (+leg, -foot) */
        /*  	(3) = hip torque (+inner, -outer) */
        /*  */
        /*  f = [2, 1] = disturbance force applied at hip: */
        /*    (1) = fx = horizontal component */
        /*    (2) = fy = vertical component */
        /*  */
        /*    qc = [2 x 1] contact point angle relative to foot */
        /*    rc = [2 x 1] contact point radius */
        /*    ddc1 = [ 2 x 1] foot one contact point acceleration */
        /*  */
        /*  dyn = parameter struct: */
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
        /*  ddz = [6, 1] = full configuration acceleration vector */
        /*    F1 = [2 x 1] contact forces on the inner feet */
        /*    ddc0 = [2 x 1] acceleration of the contact point on the outer feet */
        /*  */
        /*    Angle Conventions: */
        /*        - All angles are in absolute reference frame */
        /*        - zero corresponds to: */
        /*            (legs) = ankle joint directly below hip joint */
        /*            (feet) = ankle joint directly below virtual foot center */
        /*  */
        /*  x = z(1); */
        /*  y = z(2); */
        /*  phi0 = z(3); */
        /* 'dynamics_s1:58' phi1 = z(4); */
        /* 'dynamics_s1:59' th0 = z(5); */
        /* 'dynamics_s1:60' th1 = z(6); */
        /*  dx = dz(1); */
        /*  dy = dz(2); */
        /*  dphi0 = dz(3); */
        /* 'dynamics_s1:65' dphi1 = dz(4); */
        /* 'dynamics_s1:66' dth0 = dz(5); */
        /* 'dynamics_s1:67' dth1 = dz(6); */
        /* 'dynamics_s1:69' u0 = u(1); */
        /* 'dynamics_s1:70' u1 = u(2); */
        /* 'dynamics_s1:71' uHip = u(3); */
        /* 'dynamics_s1:73' fx = f(1); */
        /* 'dynamics_s1:74' fy = f(2); */
        /*  Contact points for each foot */
        /*  qc0 = qc(1); */
        /* 'dynamics_s1:78' qc1 = qc(2); */
        /*  rc0 = rc(1); */
        /* 'dynamics_s1:80' rc1 = rc(2); */
        /*  Acceleration of the virtual center of the foot */
        /* 'dynamics_s1:83' ddp1c_x = ddc1(1); */
        /* 'dynamics_s1:84' ddp1c_y = ddc1(2); */
        /* 'dynamics_s1:86' [M_s1,f_s1] = autoGen_dynamics_s1(... */
        /* 'dynamics_s1:87'     th0,th1,phi1,... */
        /* 'dynamics_s1:88'     dth0,dth1,dphi1,... */
        /* 'dynamics_s1:89'     u0,u1,uHip,... */
        /* 'dynamics_s1:90'     fx, fy,... */
        /* 'dynamics_s1:91'     qc1,rc1,ddp1c_x,ddp1c_y,... */
        /* 'dynamics_s1:92'     dyn.g,dyn.l,dyn.c,dyn.m,dyn.I,dyn.Ifoot,dyn.b); */
        /* AUTOGEN_DYNAMICS_S1 */
        /*     [M_S1,F_S1] = AUTOGEN_DYNAMICS_S1(TH0,TH1,PHI1,DTH0,DTH1,DPHI1,U0,U1,UHIP,FX,FY,QC1,RC1,DDP1C_X,DDP1C_Y,G,L,C,M,I,IFOOT,B) */
        /*     This function was generated by the Symbolic Math Toolbox version 6.3. */
        /*     30-Jan-2016 13:22:40 */
        /* 'autoGen_dynamics_s1:8' t2 = cos(th0); */
        t2 = cos(state[4]);

        /* 'autoGen_dynamics_s1:9' t3 = sin(th0); */
        t3 = sin(state[4]);

        /* 'autoGen_dynamics_s1:10' t4 = cos(th1); */
        t4 = cos(state[5]);

        /* 'autoGen_dynamics_s1:11' t5 = sin(th1); */
        t5 = sin(state[5]);

        /* 'autoGen_dynamics_s1:12' t6 = c.^2; */
        /* 'autoGen_dynamics_s1:13' t7 = -I-m.*t6; */
        t7 = -dyn->I - dyn->m * (dyn->c * dyn->c);

        /* 'autoGen_dynamics_s1:14' t8 = phi1+qc1; */
        t8 = state[3] + qc1;

        /* 'autoGen_dynamics_s1:15' t9 = cos(t8); */
        t9 = cos(t8);

        /* 'autoGen_dynamics_s1:16' t10 = rc1.*t9; */
        t10 = rc1 * t9;

        /* 'autoGen_dynamics_s1:17' t11 = sin(t8); */
        t11 = sin(t8);

        /* 'autoGen_dynamics_s1:18' t12 = rc1.*t11; */
        t12 = rc1 * t11;

        /* 'autoGen_dynamics_s1:19' t13 = l.*t4; */
        t13 = dyn->l * t4;

        /* 'autoGen_dynamics_s1:20' t14 = l.*t5; */
        t14 = dyn->l * t5;

        /* 'autoGen_dynamics_s1:21' M_s1 = reshape([m.*-2.0,0.0,-c.*m.*t2,-c.*m.*t4,0.0,0.0,-1.0,0.0,0.0,m.*-2.0,-c.*m.*t3,-c.*m.*t5,0.0,0.0,0.0,-1.0,0.0,0.0,-Ifoot,0.0,-Ifoot,0.0,0.0,0.0,0.0,0.0,0.0,-Ifoot,0.0,-Ifoot,-t10,-t12,-c.*m.*t2,-c.*m.*t3,t7,0.0,0.0,0.0,0.0,0.0,-c.*m.*t4,-c.*m.*t5,0.0,t7,0.0,0.0,-t13,-t14,1.0,0.0,0.0,t10+t13,0.0,t10,0.0,0.0,0.0,1.0,0.0,t12+t14,0.0,t12,0.0,0.0],[8,8]); */
        b_x[0] = dyn->m * -2.0;
        b_x[1] = 0.0;
        b_x[2] = -dyn->c * dyn->m * t2;
        b_x[3] = -dyn->c * dyn->m * t4;
        b_x[4] = 0.0;
        b_x[5] = 0.0;
        b_x[6] = -1.0;
        b_x[7] = 0.0;
        b_x[8] = 0.0;
        b_x[9] = dyn->m * -2.0;
        b_x[10] = -dyn->c * dyn->m * t3;
        b_x[11] = -dyn->c * dyn->m * t5;
        b_x[12] = 0.0;
        b_x[13] = 0.0;
        b_x[14] = 0.0;
        b_x[15] = -1.0;
        b_x[16] = 0.0;
        b_x[17] = 0.0;
        b_x[18] = -dyn->Ifoot;
        b_x[19] = 0.0;
        b_x[20] = -dyn->Ifoot;
        b_x[21] = 0.0;
        b_x[22] = 0.0;
        b_x[23] = 0.0;
        b_x[24] = 0.0;
        b_x[25] = 0.0;
        b_x[26] = 0.0;
        b_x[27] = -dyn->Ifoot;
        b_x[28] = 0.0;
        b_x[29] = -dyn->Ifoot;
        b_x[30] = -t10;
        b_x[31] = -t12;
        b_x[32] = -dyn->c * dyn->m * t2;
        b_x[33] = -dyn->c * dyn->m * t3;
        b_x[34] = t7;
        b_x[35] = 0.0;
        b_x[36] = 0.0;
        b_x[37] = 0.0;
        b_x[38] = 0.0;
        b_x[39] = 0.0;
        b_x[40] = -dyn->c * dyn->m * t4;
        b_x[41] = -dyn->c * dyn->m * t5;
        b_x[42] = 0.0;
        b_x[43] = t7;
        b_x[44] = 0.0;
        b_x[45] = 0.0;
        b_x[46] = -t13;
        b_x[47] = -t14;
        b_x[48] = 1.0;
        b_x[49] = 0.0;
        b_x[50] = 0.0;
        b_x[51] = t10 + t13;
        b_x[52] = 0.0;
        b_x[53] = t10;
        b_x[54] = 0.0;
        b_x[55] = 0.0;
        b_x[56] = 0.0;
        b_x[57] = 1.0;
        b_x[58] = 0.0;
        b_x[59] = t12 + t14;
        b_x[60] = 0.0;
        b_x[61] = t12;
        b_x[62] = 0.0;
        b_x[63] = 0.0;
        memcpy(&M_s1[0], &b_x[0], sizeof(double) << 6);

        /* 'autoGen_dynamics_s1:22' if nargout > 1 */
        /* 'autoGen_dynamics_s1:23' t15 = dth0.^2; */
        t15 = state[10] * state[10];

        /* 'autoGen_dynamics_s1:24' t16 = dth1.^2; */
        t16 = state[11] * state[11];

        /* 'autoGen_dynamics_s1:25' t17 = b.*dphi1; */
        t17 = dyn->b * state[9];

        /* 'autoGen_dynamics_s1:26' t18 = dphi1.^2; */
        t18 = state[9] * state[9];

        /* 'autoGen_dynamics_s1:27' f_s1 = [-fx-c.*m.*t3.*t15-c.*m.*t5.*t16;-fy+g.*m.*2.0+c.*m.*t2.*t15+c.*m.*t4.*t16;uHip+c.*g.*m.*t3;t17-uHip+c.*g.*m.*t5;u0;t17+u1;-ddp1c_x-l.*t5.*t16-rc1.*t11.*t18;-ddp1c_y+l.*t4.*t16+rc1.*t9.*t18]; */
        /* 'dynamics_s1:94' vars = M_s1 \ f_s1; */
        vars[0] = (-dist[0] - dyn->c * dyn->m * t3 * t15) - dyn->c * dyn->m * t5
          * t16;
        vars[1] = ((-dist[1] + dyn->g * dyn->m * 2.0) + dyn->c * dyn->m * t2 *
                   t15) + dyn->c * dyn->m * t4 * t16;
        vars[2] = S0_torque[2] + dyn->c * dyn->g * dyn->m * t3;
        vars[3] = (t17 - S0_torque[2]) + dyn->c * dyn->g * dyn->m * t5;
        vars[4] = S0_torque[0];
        vars[5] = t17 + S0_torque[1];
        vars[6] = (-ddc1[0] - dyn->l * t5 * t16) - rc1 * t11 * t18;
        vars[7] = (-ddc1[1] + dyn->l * t4 * t16) + rc1 * t9 * t18;
        mldivide(M_s1, vars);

        /* 'dynamics_s1:96' ddz = vars(1:6); */
        /* 'dynamics_s1:97' F1 = vars(7:8); */
        /*  time-step */
        /* 'simulate:305' [z, dz] = symplecticEuler(z,dz,ddz,dyn.dt); */
        /*  */
        /*  This function computes the next position and velocity, using symplectic */
        /*  euler integration method. */
        /*  */
        /* 'simulate:234' dzNext = dz + h*ddz; */
        /* 'simulate:235' zNext = z + h*dzNext; */
        for (b_k = 0; b_k < 6; b_k++) {
          b_dz = state[b_k + 6] + dyn->dt * vars[b_k];
          z[b_k] = state[b_k] + dyn->dt * b_dz;
          dz[b_k] = b_dz;
        }

        /* 'simulate:307' c0 = kinematicsContact(z,dz,qc,rc,dyn); */
        /*  [p0c,p1c,dp0c,dp1c] = kinematicsContact(z,dz,qc,rc,dyn) */
        /*  */
        /*  This function computes the kinematics of the contact point on each foot */
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
        /*  */
        /*    qc = [2 x 1] contact point angle relative to foot */
        /*    rc = [2 x 1] contact point radius */
        /*    dyn = parameter struct: */
        /*     .d = eccentricity of the foot */
        /*     .l = leg length (hip joint to foot joint) */
        /*  */
        /*  OUTPUTS: */
        /*    p0c = position of outer foot contact point */
        /*    p1c = position of inner foot contact point */
        /*    dp0c = velocity of outer foot contact point */
        /*    dp1c = velocity of inner foot contact point */
        /*  */
        /*    Angle Conventions: */
        /*        - All angles are in absolute reference frame */
        /*        - zero corresponds to: */
        /*            (legs) = ankle joint directly below hip joint */
        /*            (feet) = ankle joint directly below virtual foot center */
        /*  */
        /* 'kinematicsContact:35' x = z(1); */
        /* 'kinematicsContact:36' y = z(2); */
        /* 'kinematicsContact:37' phi0 = z(3); */
        /* 'kinematicsContact:38' phi1 = z(4); */
        /* 'kinematicsContact:39' th0 = z(5); */
        /* 'kinematicsContact:40' th1 = z(6); */
        /* 'kinematicsContact:42' qc0 = qc(1); */
        /* 'kinematicsContact:43' qc1 = qc(2); */
        /* 'kinematicsContact:44' rc0 = rc(1); */
        /* 'kinematicsContact:45' rc1 = rc(2); */
        /* 'kinematicsContact:47' if nargout > 2 */
        /* 'kinematicsContact:56' if nargout <= 2 */
        /*  Calculate positions onle */
        /* 'kinematicsContact:57' [p0c,p1c] = autoGen_kinematicsContact(... */
        /* 'kinematicsContact:58'         x,y,phi0,phi1,th0,th1,... */
        /* 'kinematicsContact:59'         [],[],[],[],[],[],... */
        /* 'kinematicsContact:60'         qc0,qc1,rc0,rc1,... */
        /* 'kinematicsContact:61'         dyn.l); */
        autoGen_kinematicsContact(z[0], z[1], z[2], z[3], z[4], z[5], qc0, qc1,
          rc0, rc1, dyn->l, c0, p1c);

        /*  Check assumptions: */
        /* 'simulate:310' f1n = dot(f1,n1); */
        f1n = 0.0;
        for (b_k = 0; b_k < 2; b_k++) {
          f1n += vars[b_k + 6] * n0[b_k];
        }

        /* 'simulate:311' valid = c0(2) > groundModel(c0(1),dyn.ground) && f1n > 0; */
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
        if ((c0[1] > ((dyn->ground[0] + dyn->ground[1] * c0[0]) + dyn->ground[2]
                      * sin(6.2831853071795862 * (dyn->ground[3] * c0[0] +
                dyn->ground[4]))) + dyn->ground[5] * c0[0] * c0[0]) && (f1n >
             0.0)) {
          S1_valid = true;
        } else {
          S1_valid = false;
        }

        /*  pack up forces */
        /* 'simulate:313' f = [0;0;f1]; */
        /* 'simulate:315' stateNext = [z;dz]; */
        /* 'simulate:317' violation = -f1n; */
        /* 'simulate:171' if S1_valid */
        if (S1_valid) {
          /* 'simulate:172' stateNext = S1_stateNext; */
          for (i = 0; i < 6; i++) {
            stateNext[i] = z[i];
            stateNext[i + 6] = dz[i];
          }

          /* 'simulate:172' f = S1_f; */
          f[0] = 0.0;
          f[1] = 0.0;

          /* 'simulate:172' k = [false; true]; */
          for (i = 0; i < 2; i++) {
            f[i + 2] = vars[i + 6];
            k[i] = bv1[i];
          }

          /* 'simulate:173' torque = S1_torque; */
          /* 'simulate:173' power = S1_power; */
          /* 'simulate:173' current = S1_current; */
          for (i = 0; i < 3; i++) {
            torque[i] = S0_torque[i];
            b_power[i] = S0_power[i];
            current[i] = S0_current[i];
          }

          exitg1 = 1;
        } else {
          guard1 = true;
        }
        break;

       case 2:
        /* 'simulate:181' case 2 %double stance */
        /* double stance */
        /* 'simulate:182' ddc = [ddc0;ddc1]; */
        /* 'simulate:183' n = [n0;n1]; */
        for (b_k = 0; b_k < 2; b_k++) {
          n[b_k] = n0[b_k];
          n[b_k + 2] = n1[b_k];
        }

        /* 'simulate:184' [DS_stateNext ,DS_f, DS_valid, DS_vln, DS_torque, DS_power, DS_current]... */
        /* 'simulate:185'                 = simulate_ds(z,dz,qc,rc,ddc,n,ir, cp, cd, dyn, motor, dist); */
        /*  Run motor model: */
        /* 'simulate:273' [u, power, current] = motorModel([z;dz], ir, cp, cd, motor); */
        for (b_k = 0; b_k < 6; b_k++) {
          d_state[b_k] = state[b_k];
          d_state[b_k + 6] = state[b_k + 6];
        }

        motorModel(d_state, ir, cp, cd, motor->ank.R, motor->ank.Vc,
                   motor->ank.K, motor->ank.G, motor->ank.c1, motor->ank.c0,
                   motor->ank.mu, motor->ank.Imax, motor->ank.alpha,
                   motor->ank.xSpring, motor->ank.kSpring, &motor->hip,
                   motor->Phi, S0_torque, S0_power, S0_current);

        /*  dynamics */
        /* 'simulate:276' [ddz, f0, f1] = dynamics_ds(z,dz,u,dist,qc,rc,ddc,dyn); */
        for (b_k = 0; b_k < 2; b_k++) {
          b_ddc0[b_k] = ddc0[b_k];
          b_ddc0[b_k + 2] = ddc1[b_k];
        }

        dynamics_ds(*(double (*)[6])&state[0], *(double (*)[6])&state[6],
                    S0_torque, dist, qc, rc, b_ddc0, dyn->g, dyn->l, dyn->c,
                    dyn->m, dyn->I, dyn->Ifoot, dyn->b, dz, c0, p1c);

        /*  time-step */
        /* 'simulate:278' [z, dz] = symplecticEuler(z,dz,ddz,dyn.dt); */
        /*  */
        /*  This function computes the next position and velocity, using symplectic */
        /*  euler integration method. */
        /*  */
        /* 'simulate:234' dzNext = dz + h*ddz; */
        for (b_k = 0; b_k < 6; b_k++) {
          dz[b_k] = state[6 + b_k] + dyn->dt * dz[b_k];
        }

        /* 'simulate:235' zNext = z + h*dzNext; */
        /*  Compute the ground height at the next contact */
        /* 'simulate:281' n0 = n(1:2); */
        /* 'simulate:281' n1 = n(3:4); */
        /*  check assumptions: */
        /* 'simulate:284' fn0 = dot(n0,f0); */
        fn0 = 0.0;

        /* Normal force on foot one */
        /* 'simulate:285' fn1 = dot(n1,f1); */
        fn1 = 0.0;
        for (b_k = 0; b_k < 2; b_k++) {
          fn0 += n[b_k] * c0[b_k];
          fn1 += n[b_k + 2] * p1c[b_k];
        }

        /* Normal force on foot two */
        /* 'simulate:286' valid = fn0 > 0 && fn1 > 0; */
        if ((fn0 > 0.0) && (fn1 > 0.0)) {
          DS_valid = true;
        } else {
          DS_valid = false;
        }

        /* Collect forces: */
        /* 'simulate:289' f = [f0;f1]; */
        /* 'simulate:290' stateNext = [z;dz]; */
        /* 'simulate:292' violation = max(-fn0,-fn1); */
        /* 'simulate:186' if DS_valid */
        if (DS_valid) {
          /* 'simulate:187' stateNext = DS_stateNext; */
          for (i = 0; i < 6; i++) {
            stateNext[i] = state[i] + dyn->dt * dz[i];
            stateNext[i + 6] = dz[i];
          }

          /* 'simulate:187' f = DS_f; */
          /* 'simulate:187' k = [true; true]; */
          for (i = 0; i < 2; i++) {
            f[i] = c0[i];
            f[i + 2] = p1c[i];
            k[i] = true;
          }

          /* 'simulate:188' torque = DS_torque; */
          /* 'simulate:188' power = DS_power; */
          /* 'simulate:188' current = DS_current; */
          for (i = 0; i < 3; i++) {
            torque[i] = S0_torque[i];
            b_power[i] = S0_power[i];
            current[i] = S0_current[i];
          }

          exitg1 = 1;
        } else {
          guard1 = true;
        }
        break;

       default:
        /* 'simulate:196' otherwise */
        /* flight */
        /* 'simulate:197' [FL_stateNext, FL_f, FL_valid, FL_torque, FL_power, FL_current] = ... */
        /* 'simulate:198'                 simulate_fl(z,dz,qc,rc,ir, cp, cd, dyn, motor, dist); */
        simulate_fl(*(double (*)[6])&state[0], *(double (*)[6])&state[6], qc, rc,
                    ir, cp, cd, dyn->g, dyn->l, dyn->c, dyn->m, dyn->I,
                    dyn->Ifoot, dyn->ground, dyn->dt, motor, dist, stateNext,
                    &FL_valid, torque, b_power, current);

        /* 'simulate:199' if FL_valid */
        if (FL_valid) {
          /* 'simulate:200' stateNext = FL_stateNext; */
          /* 'simulate:200' f = FL_f; */
          for (i = 0; i < 4; i++) {
            f[i] = 0.0;
          }

          /* 'simulate:200' k = [false; false]; */
          for (i = 0; i < 2; i++) {
            k[i] = false;
          }

          /* 'simulate:201' torque = FL_torque; */
          /* 'simulate:201' power = FL_power; */
          /* 'simulate:201' current = FL_current; */
          exitg1 = 1;
        } else {
          /* 'simulate:203' elseif XX_vln == 0 */
          /* 'simulate:204' XX_stateNext = FL_stateNext; */
          /* 'simulate:204' XX_f = FL_f; */
          for (b_k = 0; b_k < 4; b_k++) {
            f[b_k] = 0.0;
          }

          /* 'simulate:205' XX_k = [true; true]; */
          for (b_k = 0; b_k < 2; b_k++) {
            k[b_k] = true;
          }

          /* 'simulate:206' XX_torque = FL_torque; */
          /* 'simulate:206' XX_power = FL_power; */
          /* 'simulate:206' XX_current = FL_current; */
          guard1 = true;
        }
        break;
      }

      if (guard1) {
        i++;
      }
    } else {
      /*  If the code reaches here, then no valid solution was found. This means */
      /*  that one of the feet must be slipping. Since we assume no-slip, we get an */
      /*  inconsistent solution. To resolve this, we relax the ground contact force */
      /*  constraint that demands the contact forces be positive (allow negative */
      /*  reaction forces until slipping stops). */
      /* 'simulate:218' stateNext = XX_stateNext; */
      /* 'simulate:219' f = XX_f; */
      /* 'simulate:220' k = XX_k; */
      /* 'simulate:221' torque = XX_torque; */
      /* 'simulate:222' power = XX_power; */
      /* 'simulate:223' current = XX_current; */
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* End of code generation (simulate.c) */
