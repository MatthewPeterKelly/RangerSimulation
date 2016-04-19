function u = ctrlActiveTracking(~,z,xi,wn,P)

% This controller attempts to actively move the motor out of the way,
% preventing oscillations and allowing nearly free swinging of the
% pendulum. It then adds additional control effort to achieve some tracking
% objective.

qf = z(1,:);
qm = z(2,:);
dqf = z(3,:);
dqm = z(4,:);

% Reference for tracking
qr = 0.0;
dqr = 0.0;


Jm = P.Jm;   % Motor inertia
Ks = P.Ks;  %Spring constant

% Torque needed to get the motor out of the way, and prevent oscillations
uStar = (Jm*wn*wn - Ks)*(qf-qm) + 2*Jm*xi*wn*(dqf-dqm);  

% Torque needed to achieve tracking objective:
Jf = P.Jf;  %Foot inertia
N = 10;   %Ratio in tracking performance
kp = Jf*(wn/N)^2;
kd = 2*Jf*xi*(wn/N);
uTrack = kp*(qr-qf) + kd*(dqr-dqf);

% Add together:
u = uStar + uTrack;

% Saturation:
uMax = P.uMax;
u(u>uMax) = uMax;
u(u<-uMax) = -uMax;

end