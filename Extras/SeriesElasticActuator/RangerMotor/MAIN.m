%MAIN - series elastic actuator
%
% Runs a simulation of a half-Ranger model, where the ankle joint is
% controlled by a series elastic actuator.
%

clc; clear;

% Physical parameters:
P.Rad = 0.2;  %Radius of the disk
P.Len = 0.81;  % distance from joint to the leg CoM
P.Ecc = 0.14;  % Distance between virtual disk center and ankle joint
P.g = 9.81;  %gravity acceleration
P.Jf = 0.0;  %Moment of inertia of the foot (disk) (mass = zero)
P.Jm = 1.6e-6;  %Motor Rotor inertia
P.Jl = 0.45;  %Moment of inertia of the leg about its CoM
P.m = 4.5;  %Mass of the leg
P.Km = 0.018;  % Motor torque constant
P.Ks = 14;  % Spring constant coupling foot to motor
P.Ga = 34;  % transmission ratio between rotor and output shaft on motor
P.C0 = 0.01;   % motor friction paramter ( constant )
P.C1 = 0.0;   % Motor friction parametr ( damping )
P.Mu = 0.1;  % motor friction torque-dep
P.Smooth = 0.1;  % Friction smoothing term
P.Cf = 0.05;   % Viscous rolling friction
P.Imax = 8;  %Current limit

%%%% NOTES: See DynSys for controller parameters

z0 = [...   %Initial angles
    0.0;  %disk angle
    0.0;  %Motor angle
    0.02];  %pendulum angle

dz0 = [...   %Initial rates
    0.0;  %disk rate
    0.0;  %Motor rate
    0.0];  %pendulum rate

tSpan = [0,1.0];

options = odeset(...
    'RelTol',1e-6,...
    'AbsTol',1e-6);

dynFun = @(t,x)( dynSys(t,x,P) );

x0 = [z0;dz0];
sol = ode45(dynFun,tSpan,x0,options);

t = linspace(tSpan(1), tSpan(2), 250);
x = deval(sol,t);
[dx, Tm] = dynFun(t,x);
z = x(1:3,:);
dz = x(4:6,:);

qf = z(1,:);
qm = z(2,:);
ql = z(3,:);

dqf = dz(1,:);
dqm = dz(2,:);
dql = dz(3,:);

Tf = -P.Ks*(qf-qm);

figure(1); clf;

subplot(3,2,1); hold on;
plot(t,qf)
plot(t,qm)
legend('qf','qm');
xlabel('time')
ylabel('angle')
title('Foot (Disk)')

subplot(3,2,3); hold on;
plot(t,dqf)
plot(t,dqm)
legend('dqf','dqm');
xlabel('time')
ylabel('rate')

subplot(3,2,5); hold on;
plot(t,Tf);
plot(t,Tm);
legend('Tf','Tm');
xlabel('time')
ylabel('current')

subplot(3,2,2); hold on;
plot(t,ql)
legend('ql');
xlabel('time')
ylabel('angle')
title('Leg (Pendulum)')

subplot(3,2,4); hold on;
plot(t,dql)
legend('dql');
xlabel('time')
ylabel('rate')


