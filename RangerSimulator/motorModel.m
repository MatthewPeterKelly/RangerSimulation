function [u, power, current] = motorModel(x, iRef, cp, cd, model_motor)
% [u, power, current] = motorModel(x, iRef, cp, cd, model_motor)
%
% Implements the low-level motor controller and motor physics model
%

angAbs = x(3:6);
rateAbs = x(9:12);
[angRel, rateRel] = getAngRel(angAbs,rateAbs, model_motor.Phi);

qh = angRel(2);
q0 = angRel(3);
q1 = angRel(4);

dqh = rateRel(2);
dq0 = rateRel(3);
dq1 = rateRel(4);

u = zeros(3,1);  %Torque to be exerted on each joint
power = zeros(3,1);  %Electrical power used by each joint
current = zeros(3,1);  %current sent to each motor


%%%% Outer Ankle Motor:
x = q0;
v = dq0;
R = model_motor.ank.R;
Vc = model_motor.ank.Vc;
K = model_motor.ank.K;
G = model_motor.ank.G;
c1 = model_motor.ank.c1;
c0 = model_motor.ank.c0;
mu = model_motor.ank.mu;
Imax = model_motor.ank.Imax;
alpha = model_motor.ank.alpha;
xSpring = model_motor.ank.xSpring;
kSpring = model_motor.ank.kSpring;
[u(1),power(1),current(1)] = RangerJointPhysics(x,v,...
    R, Vc, K, G, c1, c0, mu,Imax,alpha,...
    iRef(1), cp(1), cd(1),...
    xSpring, kSpring);

%%%% Inner Ankle Motor:
x = q1;
v = dq1;
% R = motor_model.ank.R;
% Vc = motor_model.ank.Vc;
% K = motor_model.ank.K;
% G = motor_model.ank.G;
% c1 = motor_model.ank.c1;
% c0 = motor_model.ank.c0;
% mu = motor_model.ank.mu;
% Imax = motor_model.ank.Imax;
% alpha = motor_model.ank.alpha;
% xSpring = motor_model.ank.xSpring;
% kSpring = motor_model.ank.kSpring;
[u(2),power(2),current(2)] = RangerJointPhysics(x,v,...
    R, Vc, K, G, c1, c0, mu,Imax,alpha,...
    iRef(2), cp(2), cd(2),...
    xSpring, kSpring);

%%%% Hip Motor:
x = qh;
v = dqh;
R = model_motor.hip.R;
Vc = model_motor.hip.Vc;
K = model_motor.hip.K;
G = model_motor.hip.G;
c1 = model_motor.hip.c1;
c0 = model_motor.hip.c0;
mu = model_motor.hip.mu;
Imax = model_motor.hip.Imax;
alpha = model_motor.hip.alpha;
xSpring = model_motor.hip.xSpring;
kSpring = model_motor.hip.kSpring;
[u(3),power(3),current(3)] = RangerJointPhysics(x,v,...
    R, Vc, K, G, c1, c0, mu,Imax,alpha,...
    iRef(3), cp(3), cd(3),...
    xSpring, kSpring);

end


function [T,P,I] = RangerJointPhysics(x,v,...
    R, Vc, K, G, c1, c0, mu,Imax,alpha,...
    Ir, Cp, Cd,...
    xSpring, kSpring)
%
%
% This function takes the joint commands and does it's best to match the
% physics that the robot joint will actually do.
%
% INPUTS:
%   x = joint angle
%   v = joint velocity
%   R = 1.3 ohms (terminal resistance)
%   Vc = 0.7 volts (contact voltage drop)
%   K = 0.018 Nm/A (torque constant)
%   G = {66 -> hip, 34 -> ankle} gearbox ratio 
%   c1 = 0 Nms/rad (viscous friction)
%   c0 = 0.01 Nm (constant friction)
%   mu = 0.1 (current-depentent constant friction)
%   Jm = 1.6e-6 kg m^2 (motor inertia)
%   Imax = maximum allowable current command
%   {Ir, Cp, Cd} low-level motor control parameters:
%       I = Ir - Cp*x - Cd*v
%   xSpring = rest angle of the spring
%   kSpring = linear spring constant
%   alpha = smoothing parameter
%
% OUTPUTS:
%   T = net torque produced at the joint
%   P = electrical power consumed by the motor
%

%%% Run the low-level motor controller (essientially hardware level)
I = Ir - Cp.*x - Cd.*v;

%%% Saturate the current if necessary
I(I>Imax) = Imax;
I(I<-Imax) = -Imax;

%%% Compute the net torque
signV = tanh(v/alpha);
Tf = -c1*v - c0*signV - mu*G*K*abs(I).*signV;   %Frictional terms
Te = G*K*I;    % Electrical terms
Ts = -kSpring*(x-xSpring);     % Mechanical spring
T = Te + Tf + Ts;

%%% Compute the electrical power
P = (I*R + Vc*sign(I)).*I + G*K*v.*I; 

end