function [iRef, cp, cd, uIdeal] = motorControl(est, uRef, kp, kd, qRef, dqRef, model_control)
% [iRef, cp, cd, qRef, dqRef, uIdeal] = motorControl(est, ctrl, model_control)
%
% This function takes the commands from the high-level controller and
% converts them into low-level motor commands
%

% qr = est(1);
qh = est(2);
q0 = est(3);
q1 = est(4);

% dqr = est(5);
dqh = est(6);
dq0 = est(7);
dq1 = est(8);

% Allocate memory.  [u0; u1; uHip];
iRef = zeros(3,1);
cp = zeros(3,1);
cd = zeros(3,1);
uIdeal = zeros(3,1);  %Desired motor torque 

%%%% Outer Ankles:
kSpring = model_control.ank.kSpring;
uMax = model_control.ank.uMax;
qSpring = model_control.ank.qSpring;
kMotor = model_control.ank.kMotor;
[iRef(1), cp(1), cd(1), uIdeal(1)] = RangerJointControl(...
    q0,qRef(1),dq0,dqRef(1), uRef(1), uMax, kp(1), kd(1),...
    qSpring, kSpring, kMotor);


%%%% Outer Ankles:
kSpring = model_control.ank.kSpring;
uMax = model_control.ank.uMax;
qSpring = model_control.ank.qSpring;
kMotor = model_control.ank.kMotor;
[iRef(2), cp(2), cd(2), uIdeal(2)] = RangerJointControl(...
    q1,qRef(2),dq1,dqRef(2), uRef(2), uMax, kp(2), kd(2),...
    qSpring, kSpring, kMotor);

%%%% Hip:
kSpring = model_control.hip.kSpring;
uMax = model_control.hip.uMax;
qSpring = model_control.hip.qSpring;
kMotor = model_control.hip.kMotor;
[iRef(3), cp(3), cd(3), uIdeal(3)] = RangerJointControl(...
    qh,qRef(3),dqh,dqRef(3), uRef(3), uMax, kp(3), kd(3),...
    qSpring, kSpring, kMotor);


end








function [ir, cp, cd, uSmooth] = RangerJointControl(...
    q,qRef,dq,dqRef, uRef, uMax, kp, kd,...
    qSpring, kSpring, kMotor)
% [ir, cp, cd] = RangerJointControl(...
%    x,xRef,v,vRef, uRef, uMax, kp, kd,...
%    xSpring, kSpring, kMotor)
%
% This function performs the full control stack for one of Ranger's motors,
% starting with the high-level joint commands and returning the paramters
% for the low-level motor controllers. No fancy saturation is used here.
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
uRaw = uRef + kp*(qRef-q) + kd*(dqRef-dq);

%%% Saturate using tanh() smoothing
uSmooth = uRaw;

%%% Compute gradient of the smooth controller wrt to x and v
% Note:
%   - if at origin (uSmooth == 0), then Ux == kp and Uv == kd
S = 1;
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
uLin = uSmooth + Ux.*q + Uv.*dq;

%%% Compensate for spring:
uStar = uLin - kSpring*qSpring;
kpStar = Ux - kSpring;
kdStar = Uv;

%%% Convert to electrical current offset and gains:
% Note:
%   kMotor = (motor constant K)*(gearbox ratio G)
ir = uStar/kMotor;
cp = kpStar/kMotor;
cd = kdStar/kMotor;

end











function [ir, cp, cd, uSmooth] = RangerJointControl_SmoothSaturation(...
    q,qRef,dq,dqRef, uRef, uMax, kp, kd,...
    qSpring, kSpring, kMotor)
% [ir, cp, cd] = RangerJointControl(...
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
uRaw = uRef + kp*(qRef-q) + kd*(dqRef-dq);

%%% Saturate using tanh() smoothing
uSmooth = uMax*tanh( uRaw/uMax );

%%% Compute gradient of the smooth controller wrt to x and v
% Note:
%   - if at origin (uSmooth == 0), then Ux == kp and Uv == kd
S = 1 - tanh(uRaw/uMax).^2;
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
uLin = uSmooth + Ux.*q + Uv.*dq;

%%% Compensate for spring:
uStar = uLin - kSpring*qSpring;
kpStar = Ux - kSpring;
kdStar = Uv;

%%% Convert to electrical current offset and gains:
% Note:
%   kMotor = (motor constant K)*(gearbox ratio G)
ir = uStar/kMotor;
cp = kpStar/kMotor;
cd = kdStar/kMotor;

end