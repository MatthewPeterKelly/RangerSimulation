function [ddq, f] = motorDynamics(qEnd,q,dq,I,P)
% [ddq, f] = motorDynamics(qEnd,dqEnd,q,dq,I,P)
%
% This function computes the dynamics of Ranger's ankle motor, as if both
% the stance leg and the stance ankle were fixed. This system has one
% degree of freedom: the rotor angle of the motor.
%
%

% Unpack parameters:
Smooth = P.Smooth;
C1 = P.C1;
C0 = P.C0;
Mu = P.Mu;
Ga = P.Ga;
Km = P.Km;
Ks = P.Ks;
Jm = P.Jm;

% Smooth signum and abs functions:
Sgn_dq = tanh(dq/Smooth);
Abs_I = I.*tanh(I/Smooth);

% Frictional torque term:
Tf = C1*dq + C0*Sgn_dq + Mu*Ga*Km*Abs_I.*Sgn_dq;

% Torque due to tendon compliance:
Ts = -Ks*(q-qEnd);

% Electrical torque:
Te = Ga*Km*I;

% Force balance:
ddq = (Te + Ts - Tf)./(Ga*Ga*Jm); 

% Force applied to the hard stop:
f = -Ts;

end
