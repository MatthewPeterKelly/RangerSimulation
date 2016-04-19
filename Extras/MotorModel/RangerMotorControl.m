function [ir, cp, cd] = RangerMotorControl(...
    x,xRef,v,vRef, uRef, uMax, kp, kd,...
    xSpring, kSpring, kMotor)
% [ir, cp, cd] = RangerMotorControl(...
%    x,xRef,v,vRef, uRef, uMax, kp, kd,...
%    xSpring, kSpring, kMotor)
%
% This function performs the full control stack for one of Ranger's motors,
% starting with the high-level joint commands and returning the paramters
% for the low-level motor controllers.
%
% INPUTS:
%   x = joint position
%   xRef = joint target position
%   v = joint rate
%   vRef = joint target rate
%   uRef = feed-forward joint torque
%   uMax = maximum allowable joint torque
%   kp = proportional gain near origin
%   kd = derivative gain near origin
%   xSpring = joint angle when spring force is zero (rest angle)
%   kSpring = linear spring constant 
%   kMotor = K*G; % (torque after gearbox) = kMotor*(current)
%
% OUTPUTS:
%   ir = reference motor current
%   cp = proportional current gain
%   cd = derivative current gain
%
% NOTES: Read source code for additional details
%

%%% Simple feed-forward pd-controller
uRaw = uRef + kp*(xRef-x) + kd*(vRef-v);

%%% Saturate using atan() smoothing
uSmooth = uMax*(2/pi)*atan( (pi/2)*(uRaw/uMax) );

%%% Compute gradient of the smooth controller wrt to x and v
% Note:
%   - if at origin (uSmooth == 0), then Ux == kp and Uv == kd
S = (4*uMax^2) / (pi^2*uSmooth^2 + 4*uMax^2);
Ux = S*kp;
Uv = S*kd;

%%% Write in compressed linear form (as used by low-level controller)
% Example:
%   u = u0 + a*(x0-x) + b*(v0-v);
%   u = [u0+a*x0+b*v0] - [a]*x - [b]*v
% Note: 
%   - Linearize about the current state (not the reference)
%   - For nearby points, the controller is evaluated by:
%       u = uLin - Ux*xLocal + Uv*vLocal
%   - Notice that uLin == uSmooth, if xLocal == x and vLocal = v
uLin = uSmooth + Ux*x + Uv*v;  

%%% Compensate for spring:
uStar = uLin - kSpring*xSpring;
kpStar = Ux - kSpring;
kdStar = Uv;  

%%% Convert to electrical current offset and gains:
% Note:
%   kMotor = (motor constant K)*(gearbox ratio G)
ir = uStar/kMotor;
cp = kpStar/kMotor;
cd = kdStar/kMotor;

end