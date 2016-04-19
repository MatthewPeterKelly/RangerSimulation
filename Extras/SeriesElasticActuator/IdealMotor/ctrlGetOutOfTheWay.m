function u = ctrlGetOutOfTheWay(~,z,xi,wn,P)

% This controller attempts to actively move the motor out of the way,
% preventing oscillations and allowing nearly free swinging of the pendulum

qf = z(1,:);
qm = z(2,:);
dqf = z(3,:);
dqm = z(4,:);

Jm = P.Jm;   % Motor inertia
Ks = P.Ks;  %Spring constant

u = (Jm*wn*wn - Ks)*(qf-qm) + 2*Jm*xi*wn*(dqf-dqm);

end