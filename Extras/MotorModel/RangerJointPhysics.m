function [T,P,I] = RangerJointPhysics(x,v,a,...
    R, Vc, K, G, c1, c0, mu, Jm,Imax,...
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
%   a = joint acceleration
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
Tf = -c1*v - c0*ssign(v) - mu*G*K*abs(I).*ssign(v);   %Frictional terms
Te = G*(K*I - Jm*G*a);    % Electrical terms
Ts = -kSpring*(x-xSpring);     % Mechanical spring
T = Te + Tf + Ts;

%%% Compute the electrical power
if nargout > 1
   P = (I*R + Vc*ssign(I)).*I + G*K*v.*I; 
end

end

function y = ssign(x)
% A smooth version of the signum function
%
% alpha == 0.02   <--> smooths inputs on [-0.1, 0.1]

alpha = 0.02;  % Smoothing parameter
y = tanh(x/alpha);

end