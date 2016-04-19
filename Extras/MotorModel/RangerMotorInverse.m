function [I, flag] = RangerMotorInverse(omega, alpha, T, P)
%
% This function implements a slightly simplified version of the Ranger
% motor mode, as described in the appendix of the main Ranger paper. Rather
% that return a torque and power, it computes the approximate current
% required to produce the desired torque.
%
% INPUTS:
%   omega = (rad/s) rotational speed after gear box 
%   alpha = (rad/s^S) rotational acceleration after geat box

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
%   I = required current into motor
%    
%
%
% NOTES:
%   Here I assume that:
%
%   1) Static and dynamic friction are equivilent, and that Tf is zero if
%   the motor is not spinning.  c0d == c0d == c0
%    


%%%% Compute the current required to produce the desired torque:
% Tf = C1.*omega + C0*sign(omega) + mu*G*K*abs(I).*sign(omega); % Frictional Torque:
% T = G*(K*I - Jm*G*alpha) - Tf; % Torque out of gear box:
%
%
% T = G*(K*I - Jm*G*alpha) - (C1.*omega + C0*sign(omega) + mu*G*K*abs(I).*sign(omega));
%
% T = (G*K)*I   -  mu*G*K*abs(I).*sign(omega)  - G*Jm*G*alpha - C1*omega - C0*sign(omega);
%
%
% T = (G*K)*Ipos   -  mu*G*K*Ipos.*sign(omega)  - G*Jm*G*alpha - C1*omega - C0*sign(omega);
% T = (G*K)*Ineg   +  mu*G*K*Ineg.*sign(omega)  - G*Jm*G*alpha - C1*omega - C0*sign(omega);
%
%
% Ipos*(G*K - mu*G*K*sign(omega)) = T + G*Jm*G*alph + C1*omega + C0*sign(omega)
% Ineg*(G*K + mu*G*K*sign(omega)) = T + G*Jm*G*alph + C1*omega + C0*sign(omega)
%
%%%%

num = T + P.G*P.Jm*P.G*alpha + P.C1*omega + P.C0*sign(omega);
denPos = (P.G*P.K - P.mu*P.G*P.K*sign(omega));
denNeg = (P.G*P.K + P.mu*P.G*P.K*sign(omega));

Ipos = num./denPos;
Ineg = num./denNeg;
Ifail = num./(P.G*P.K);   %Something went wrong! Neglect friction.

I = zeros(size(Ipos));
cPos = Ipos > 0 & Ineg > 0;
cNeg = Ipos <= 0 & Ineg <= 0;
cFail = ~cPos & ~cNeg;

I(cPos) = Ipos(cPos);
I(cNeg) = Ineg(cNeg);
I(cFail) = Ifail(cFail);

flag = 1*cPos + 2*cNeg + 4*cFail;

end