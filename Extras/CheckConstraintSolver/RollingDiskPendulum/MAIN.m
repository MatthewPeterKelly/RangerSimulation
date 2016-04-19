% MAIN - Contact Simulation - Sanity check on rolling solver:
%
% Runs the simulation first with the minimal coordinates solution, and then


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

DT = 1*[1e-4,1e-3, 1e-2];

% Initial angles and rates:
P.q0 = (pi/180)*0;
P.q1 = (pi/180)*179;
P.dq0 = 0;
P.dq1 = 0;

% Set up time stepping:
P.tSpan = [0, 1.4];


figure(102); clf;
[t,q0,q1,dq0,dq1,Fx,Fy,KE,PE] = simulate_minimal(P);
plotSoln(t,q0,q1,dq0,dq1,Fx,Fy,KE,PE);
E = KE+PE; energy_error = max(abs(E-mean(E)));
fprintf('Maximum deviation in energy: %6.6g\n',energy_error);

P.dt = DT(1);
[t,q0,q1,dq0,dq1,Fx,Fy,KE,PE] = simulate_contact(P);
plotSoln(t,q0,q1,dq0,dq1,Fx,Fy,KE,PE);
E = KE+PE; energy_error = max(abs(E-mean(E)));
fprintf('Dt = %6.6g, Maximum deviation in energy: %6.6g\n',P.dt,energy_error);

P.dt = DT(2);
[t,q0,q1,dq0,dq1,Fx,Fy,KE,PE] = simulate_contact(P);
plotSoln(t,q0,q1,dq0,dq1,Fx,Fy,KE,PE);
E = KE+PE; energy_error = max(abs(E-mean(E)));
fprintf('Dt = %6.6g, Maximum deviation in energy: %6.6g\n',P.dt,energy_error);

P.dt = DT(3);
[t,q0,q1,dq0,dq1,Fx,Fy,KE,PE] = simulate_contact(P);
plotSoln(t,q0,q1,dq0,dq1,Fx,Fy,KE,PE);
E = KE+PE; energy_error = max(abs(E-mean(E)));
fprintf('Dt = %6.6g, Maximum deviation in energy: %6.6g\n',P.dt,energy_error);
