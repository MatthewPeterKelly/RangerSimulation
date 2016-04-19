function I = motorControl(q,dq,fRef,P)
% I = motorControl(q,dq,fRef,P)
%
% This function runs a force controller that tracks some desired force to
% be applied to the hard stop at the far end of the tendon.
%

% Unpack physical parameters:
Ks = P.Ks;
Ga = P.Ga;
Jm = P.Jm;
Km = P.Km;
M = Ga*Ga*Jm;   %Effective motor inertia
K = Ga*Km;  % Conversion between torque and current;
% M*ddq = K*I + f   % System dynamics
% f = Ks*q;

% Compute the desired motor position:
q0 = fRef/Ks;
dq0 = 0.0;  %Assume static target for now

% Use a pd controller to compute the desired acceleration of motor:
wn = 2*pi*P.wn;
xi = P.xi;
kp = wn*wn;
kd = 2*xi*wn;
ddq = kp*(q0 - q) + kd*(dq0 - dq);
I_motor = M*ddq/K;

% Feed-forward the expected DC current needed to maintain position:
I_static = fRef/K;

% Open loop control, no dynamics:
I = I_static + I_motor;

end