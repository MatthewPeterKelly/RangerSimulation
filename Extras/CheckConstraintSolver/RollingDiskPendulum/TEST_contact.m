% MAIN - Contact Simulation - Sanity check on rolling solver:
%
%

clc; clear;

param.m0 = 0.01;
param.m1 = 4.96;
param.I0 = 0.004;
param.I1 = 0.45;
param.g = 9.81;
param.a = 0.14;
param.b = 0.81;
param.r = 0.2;
param.slope = 0.0;
param.dt = 0.05;

% Initial angles and rates:
param.q0 = (pi/180)*0;
param.q1 = (pi/180)*179;
param.dq0 = 0;
param.dq1 = 0;

% Set up time stepping:
param.tSpan = [0, 1.4];

%%%% THE KEY LINE:
[t,q0,q1,dq0,dq1,Fx,Fy,KE,PE,p0,pStar,p1] = simulate_contact(param);
E = KE+PE;
energy_error = max(E) - min(E);
fprintf('Maximum deviation in energy: %6.6g\n',energy_error);

%%%% Animation: 
data = [p0;pStar;p1];
Anim.plotFunc = @(t,x)( drawDiskPendulum(t,x,param) );
Anim.speed = 0.5;
Anim.figNum = 100;
Anim.verbose = false;
animate(t,data,Anim);

%%%% Plots:
figure(101); clf;
plotSoln(t,q0,q1,dq0,dq1,Fx,Fy,KE,PE);


