function [dx, Ia] = dynSys(t,x,P)

z = x(1:3,:);
dz = x(4:6,:);

qf = z(1,:);
qm = z(2,:);
ql = z(3,:);

dqf = dz(1,:);
dqm = dz(2,:);
dql = dz(3,:);

Ks = P.Ks;
Km = P.Km;
Jm = P.Jm;
Ga = P.Ga;  

%%%% Controller

% Attempt to hold the disk stationary at the origin
kp = 1.0;
kd = 0.1;
qr = 0.0;
dqr = 0.0;

% % Attempt to hold the angle between the disk and the leg:
% kp = 5;
% kd = 0.1;
% qr = 0.3;
% dqr = 0;

uTracking = kp*(qr-qf) + kd*(dqr-dqf);   %Tracking controller

uSpring = -Ks*(qf-qm);  %Torque on foot due to spring

uDamping = 2*sqrt(Ks*Jm*Ga*Ga)*(dqf-dqm);   %Add critical damping between foot and motor

Ia = (uTracking + uDamping - uSpring)/(Ga*Km);   %Current to send to motors


%%%% Saturation
Imax = P.Imax;
Ia( Ia > Imax ) = Imax;
Ia( Ia < -Imax) = -Imax;

%%%% Run the dynamics:

ddz = dynamics(z,dz,Ia,P);
dx = [dz;ddz];

end
