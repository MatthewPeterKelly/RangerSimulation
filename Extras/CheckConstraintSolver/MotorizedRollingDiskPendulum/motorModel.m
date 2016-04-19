function [torque, power, current] = motorModel(q0,q1,dq0,dq1, Ir, Cp, Cd, param)

% Joint kinematics:
% x = joint angle (relative)
% v = joint rate (relative)

x = q0 - q1;
v = dq0 - dq1;

% For now, ignore the joint spring:
xSpring = 0.0;
kSpring = 0.0;

K = param.K;
R = param.R;
Vc = param.Vc;
G = param.G;
c1 = param.c1;
c0 = param.c0;
mu = param.mu;
Imax = param.Imax;
alpha = param.alpha;
[torque, power, current] = RangerJointPhysics(x,v,...
    R, Vc, K, G, c1, c0, mu,Imax,alpha,...
    Ir, Cp, Cd,...
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