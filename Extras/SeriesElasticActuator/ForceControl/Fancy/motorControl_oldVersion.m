function I = motorControl_oldVersion(qEnd,dqEnd,q,dq,fRef,P)
% I = motorControl_oldVersion(qEnd,dqEnd,q,dq,fRef,P)
%
% This function runs a force controller that tracks some desired force to
% be applied to the hard stop at the far end of the tendon.
%


%%%% NOTES:
%
% This sort of works, but it does not track

% Unpack physical parameters:
Ks = P.Ks;
Ga = P.Ga;
Jm = P.Jm;
Km = P.Km;
M = Ga*Ga*Jm;   %Effective motor inertia
K = Ga*Km;  % Conversion between torque and current;
% M*ddq = K*I + f   % System dynamics
% f = Ks*(q-qEnd);

% Critical damping:
I_damp = 2*sqrt(Ks*M)*(dqEnd-dq);

% Compensate for spring:
I_spring = Ks*(q-qEnd);
I_spring = 0*I_spring;  % I_spring does not seem to be helping...

% Tracking control:
I_track = fRef/K;

% Open loop control, no dynamics:
I = I_track + I_damp - I_spring;

% Current Saturation:
Imax = P.Imax;
I(I>Imax) = Imax;
I(I<-Imax) = -Imax;

end