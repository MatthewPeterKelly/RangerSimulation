function [T, Power] = RangerMotor(omega, alpha, I, P)
%
% This function implements a slightly simplified version of the Ranger
% motor mode, as described in the appendix of the main Ranger paper.
%
% INPUTS:
%   omega = (rad/s) rotational speed after gear box 
%   alpha = (rad/s^S) rotational acceleration after geat box
%   I = current into motor
%   P = struct of paramaters
%       .R = 1.3 ohms (terminal resistance)
%       .Vc = 0.7 volts (contact voltage drop)
%       .K = 0.018 Nm/A (torque constant)
%       .C1 = 0 Nms/rad (viscous friction)
%       .C0 = 0.01 Nm (constant friction)
%       .mu = 0.1 (current-depentent constant friction)
%       .Jm = 1.6e-6 kg m^2 (motor inertia)
%       .G = {66 -> hip, 34 -> ankle} gearbox ratio
%
% OUTPUTS:
%   T = (Nm) torque at the output of the gear box
%   Power = (W) electrical power used by the motor
%
%
% NOTES:
%   Here I assume that:
%
%   1) Static and dynamic friction are equivilent, and that Tf is zero if
%   the motor is not spinning.  c0d == c0d == c0
%    

% Frictional Torque:
Tf = P.C1.*omega + P.C0*sign(omega) + P.mu*P.G*P.K*abs(I).*sign(omega);

% Torque out of gear box:
T = P.G*(P.K*I - P.Jm*P.G*alpha) - Tf;

% Electrical Power:
Power = (I*P.R + P.Vc*sign(I)).*I + P.G*P.K*omega.*I;

end