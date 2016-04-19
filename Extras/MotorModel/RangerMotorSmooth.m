function [T, P] = RangerMotorSmooth(omega, I, P)
%
% This function computes a smooth approximation of the ranger motor model
% for use in optimization.
%
% INPUTS:
%   omega = (rad/s) rotational speed after gear box 
%   I = current into motor
%   P = struct of paramaters
%
% OUTPUTS:
%   T = (Nm) torque at the output of the gear box
%   P = (W) electrical power used by the motor
%       .R = 1.3 ohms (terminal resistance)
%       .Vc = 0.7 volts (contact voltage drop)
%       .K = 0.018 Nm/A (torque constant)
%       .C1 = 0 Nms/rad (viscous friction)
%       .C0 = 0.01 Nm (constant friction)
%       .mu = 0.1 (current-depentent constant friction)
%       .Jm = 1.6e-6 kg m^2 (motor inertia)
%       .G = {66 -> hip, 34 -> ankle} gearbox ratio
%
%   P.smoothing = smoothing parameter (~0.01 for Ranger optimization)
%
%
% NOTES:
%   Here I assume that:
%
%   1) Static and dynamic friction are equivilent, and that Tf is zero if
%   the motor is not spinning.  c0d == c0d == c0
%    
%   2) Assume that the acceleration terms are neglegible (steady state)
%

%%% True versions of the functions:
% signOmega = sign(omega);
% absI = abs(I);
% signI = sign(I);

alpha = P.smoothing;
signOmega = tanh(omega/alpha);
absI = sqrt(I.*I + alpha*alpha);
signI = tanh(I/alpha);

% Frictional Torque:
Tf = P.C1.*omega + P.C0*signOmega + P.mu*P.G*P.K*absI.*signOmega;

% Torque out of gear box:
T = P.G*P.K*I - Tf;

% Electrical Power:
P = (I*P.R + P.Vc*signI).*I + P.G*P.K*omega.*I;

end