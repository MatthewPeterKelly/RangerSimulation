% TEST - minimal coordinate version of disk-pendulum system
%
%

clc; clear;

P.m0 = 0.01;
P.m1 = 4.96;
P.I0 = 0.004;
P.I1 = 0.45;
P.g = 9.81;
P.a = 0.14;
P.b = 0.81;
P.r = 0.2;
P.slope = 0.0;
P.dt = 0.05;

% Initial angles and rates:
P.q0 = (pi/180)*0;
P.q1 = (pi/180)*179;
P.dq0 = 0;
P.dq1 = 0;

% Set up time stepping:
P.tSpan = [0, 1.4];

[t,q0,q1,dq0,dq1,Fx,Fy,KE,PE,p0,pStar,p1] = simulate_minimal(P);

% Compute max energy error:
E = KE+PE;
energy_error = max(abs(E-mean(E)));
fprintf('Maximum deviation in energy: %6.6g\n',energy_error);

%%%% Animation: 
data = [p0;pStar;p1];
Anim.plotFunc = @(t,x)( drawDiskPendulum(t,x,P) );
Anim.speed = 0.5;
Anim.figNum = 100;
Anim.verbose = false;
animate(t,data,Anim);


%%%% Plots:
figure(102); clf;
plotSoln(t,q0,q1,dq0,dq1,Fx,Fy,KE,PE);


