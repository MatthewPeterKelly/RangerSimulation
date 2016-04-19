function u = ctrlTrackMotor(t,z,kp,kd)

% Attempts to track a reference angle on the motor

x = z(2,:);
v = z(4,:);

[xRef, vRef] = trigRef(t);

u = kp*(xRef - x) + kd*(vRef - v);

end

function [xRef, vRef] = trigRef(t)

a = 0.2;   %Amplitude
f = 1.0;   %Frequency (Hz)

xRef = a*sin(2*pi*f*t);
vRef = 2*pi*f*a*cos(2*pi*f*t);

end