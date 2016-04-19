function u = ctrlTrackForce(t,z,kp,kd,P)

% Attempts to track a force reference on the output

qf = z(1,:);
qm = z(2,:);
dqf = z(3,:);
dqm = z(4,:);

[fRef, dfRef] = trigRef(t);

% Compute the force:
f = -P.Ks*(qf-qm);
df = -P.Ks*(dqf-dqm);

% Force control:
u = kp*(fRef-f) + kd*(dfRef-df);

end

function [xRef, vRef] = trigRef(t)

a = 1.0;   %Amplitude
f = 1.0;   %Frequency (Hz)

xRef = a*sin(2*pi*f*t);
vRef = 2*pi*f*a*cos(2*pi*f*t);

end