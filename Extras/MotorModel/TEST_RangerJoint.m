% TEST_RangerJoint.m
%
% This script tests both RangerJointController and RangerJointPhysics


kp = 80;
kd = 20;

uMax = 4;    % 3.0 steady state, 4.5 max

xRef = 0.0;
vRef = 0.0;

x0 = -0.5;
v0 = 0.0;

dt = 1/2000;   %Low level controller @ 2000 Hz
tSpan = [0, 2];
nGrid = ceil(diff(tSpan)/dt);
t = zeros(1,nGrid);
x = zeros(1,nGrid);
v = zeros(1,nGrid);
Ir = zeros(1,nGrid);
Cp  = zeros(1,nGrid);
Cd = zeros(1,nGrid);
T = zeros(1,nGrid);
P = zeros(1,nGrid);
I = zeros(1,nGrid);

nSubStep = 4;  %High level controller runs at 500 Hz

x(1) = x0;
v(1) = v0;
a = 0.0;   %Neglect acceleration terms for now

uRef = 0.0;
xSpring = 0.0;
kSpring = 8.045;   %Ranger hip spring
kMotor = 1.188;  %Ranger hip motor constant, including gearbox

xSpring_err = 0.0;
kSpring_err = 0.0;
kMotor_err = 0.0;

R = 1.3; % ohms (terminal resistance)
Vc = 0.7; % volts (contact voltage drop)
K = 0.018; % Nm/A (torque constant)
G = 66; % hip gearbox ratio
c1 = 0; % Nms/rad (viscous friction)
c0 = 0; %.01 Nm (constant friction)
mu = 0.1; % (current-depentent constant friction)
Jm = 1.6e-6; % kg m^2 (motor inertia)
Imax = 8.0;  %A = saturation current in motor


for i=1:nGrid
    
    % Run the high-level controller
    if mod(i-1,nSubStep)==0
        [Ir(i), Cp(i), Cd(i)] = RangerJointControl(...
            x(i),xRef,v(i),vRef, uRef, uMax, kp, kd,...
            xSpring+xSpring_err,...
            kSpring+kSpring_err,...
            kMotor+kMotor_err);
    else
        Ir(i) = Ir(i-1);
        Cp(i) = Cp(i-1);
        Cd(i) = Cd(i-1);
    end
    
    % Run the low level controller and physics
    [T(i),P(i),I(i)] = RangerJointPhysics(x(i),v(i),a,...
        R, Vc, K, G, c1, c0, mu, Jm,Imax,...
        Ir(i), Cp(i), Cd(i),...
        xSpring, kSpring);
    if i<nGrid
        t(i+1) = t(i) + dt;
        v(i+1) = v(i) + dt*T(i);
        x(i+1) = x(i) + dt*v(i+1);  %Symplectic euler
    end
end

figure(2); clf;
title('Successive linearization')

subplot(3,2,1);
plot(t,x)
xlabel('t');
ylabel('x');

subplot(3,2,3);
plot(t,v)
xlabel('t');
ylabel('v');

subplot(3,2,5);
plot(t,T)
xlabel('t');
ylabel('torque');

subplot(3,2,2);
plot(t,P)
xlabel('t');
ylabel('power');

subplot(3,2,6);
plot(t,I)
xlabel('t');
ylabel('current');
