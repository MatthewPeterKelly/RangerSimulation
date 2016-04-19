% A simple script to load parameters.

%Physical paramters:
P.m0 = 0.01;
P.m1 = 4.96;
P.I0 = 0.004;
P.I1 = 0.45;
P.g = 9.81;
P.a = 0.14;
P.b = 0.81;
P.r = 0.2;
P.slope = 0.0;
P.dt = 0.002;

% Motor parameters:
P.R = 1.3; %ohms (terminal resistance)
P.Vc = 0.7; %volts (contact voltage drop)
P.G = 34;%{66 -> hip, 34 -> ankle} gearbox ratio
P.c1 = 0.0; %Nms/rad (viscous friction)
P.c0 = 0.01; %Nm (constant friction)
P.mu = 0.1; %(current-depentent constant friction)
P.Imax = 8.0; %maximum allowable motor current (3.1 continuous, 8.0 peak)
P.alpha = 0.5;  %Smoothing parameter for sign() function

% Initial angles and rates:
P.q0 = (pi/180)*0;
P.q1 = (pi/180)*179.5;
P.dq0 = 0;
P.dq1 = 0;

% Controller:
P.kp = 4;
P.kd = 2;

% Set up time:
P.tSpan = [0, 0.8];