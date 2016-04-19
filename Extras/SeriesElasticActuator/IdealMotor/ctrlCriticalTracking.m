function u = ctrlCriticalTracking(~,z,xi,kp,kd,P)

% Adds an artificial damping term to achieve a critical damping between the
% motor and the end effector, and then uses a standard pd controller


qf = z(1,:);
qm = z(2,:);
dqf = z(3,:);
dqm = z(4,:);

% Reference for tracking
qr = 0.2;
dqr = 0.0;

Jm = P.Jm;   % Motor inertia
Ks = P.Ks;  %Spring constant

% Artificial critical damping between motor and actuator
uStar = 2*xi*sqrt(Ks*Jm)*(dqf-dqm);

% Torque needed to achieve tracking objective:
uTrack = kp*(qr-qf) + kd*(dqr-dqf);

% Force already present due to spring:
uSpring = -kp*(qf-qm);

% Add together:
u = uStar + uTrack - uSpring;

% Saturation:
uMax = P.uMax;
u(u>uMax) = uMax;
u(u<-uMax) = -uMax;


end