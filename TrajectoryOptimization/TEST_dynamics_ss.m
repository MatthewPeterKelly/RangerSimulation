%TEST_dynamics_ss.m
%
% This script tests the single stance dynamics
clc; clear;

%%%% Parameters %%%%
p.g = 9.81; % acceleration due to gravity
p.l = 0.96; % leg length (hip joint to foot joint)
p.d = 0.14; % distance between foot joint and virtual center of foot
p.r = 0.2; % radius of circular arc on foot
p.c = 0.15; % distance along leg from hip to CoM
p.w = 0; % distance off leg from main axis to CoM
p.m = 4.96; % mass of each leg
p.I = 0.45; % moment of inertia of the leg about its center of mass
p.Ifoot = 0.05; % moment of inertia of the foot about the foot joint

p.angleToe = 30*(pi/180);
p.angleHeel = -5*(pi/180);

% Open-loop dynamics, zero input:
rhs = @(t,x) ( [x(5:8); dynamics_ss(x(1:4),x(5:8),zeros(3,1),p)] );

tSpan = [0,0.25];
z0 = [
    0;0; 
    0.2; -0.19];
dz0 = [
    0.1;0.5;
    -2.3;2.1];  

sol = ode45(rhs,tSpan,[z0;dz0]);


t = linspace(tSpan(1), tSpan(2), 200)';
x = deval(sol,t);
z = x(1:4,:);
dz = x(5:8,:); 
u = zeros(3,length(t));

[pHip,p1,p2,dpHip,dp1,dp2] = kinematics_ss(z,dz,p);

ddz = dynamics_ss(z,dz,u,p);
f1 = getContactForce_ss(z,dz,ddz,p);
f2 = zeros(size(f1));
f = [f1;f2];

%%%% Plot and Animate
traj.config.dyn = p;
traj.phase = makeStruct(t,z,dz,u,f,pHip,p1,p2,dpHip,dp1,dp2);
plotTraj(traj);

X = [z;pHip;p1;p2];
P.plotFunc = @(t,z,P)drawRanger(z,p); 
P.speed = 0.1;
P.figNum = 162;
animate(t,X,P);
