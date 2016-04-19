function u = ctrlTrackSmart(t,z,kp,kd,md,P)

% Attempts to track a reference angle on the motor

qf = z(1,:);
qm = z(2,:);
dqf = z(3,:);
dqm = z(4,:);

[qRef, dqRef] = trigRef(t);

% Desired force on the end effector:
uDes = kp*(qRef-qf) + kd*(dqRef-dqf);

% Compensate for spring, add some damping:
uSpring = -P.Ks*(qf-qm);
uDamp = md*(dqf-dqm);
u = uDes + uDamp - uSpring;

end

function [xRef, vRef] = trigRef(t)

a = 0.2;   %Amplitude
f = 1.0;   %Frequency (Hz)

xRef = a*sin(2*pi*f*t);
vRef = 2*pi*f*a*cos(2*pi*f*t);

end