function [Iref, Cp, Cd] = controller(q0,q1,dq0,dq1,param)
% [Iref, Cp, Cd] = controller(q0,q1,qd0,dq1,param)
%
% This controller attempts to hold the foot level while the pendulum
% rotates around it.
%
% Returns motor current commands

% Relative joint angle:
% x = q0 - q1;
% v = dq0 - dq1;

% What is the desired relative angle at the joint to hold the disk level?
%   q0 -> 0    xDes = -q1;
%   dq0 -> 0   vDes = -dq1;
%

% x = q0 - q1;
% v = dq0 - dq1;
xRef = -q1;
vRef = -dq1;
kp = param.kp;
kd = param.kd;

% Now convert to special controller form:
% u = uRef + kp*(xRef - x) + kd*(vREf - v)
% u = [uRef + kp*xRef + kd*vRef] - x*kp - v*kd;

% And finally, convert from torque to current:
Iref = (kp*xRef + kd*vRef)/param.kMotor;
Cp = kp/param.kMotor;
Cd = kd/param.kMotor;

end