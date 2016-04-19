function [angRel, rateRel] = getAngRel(angAbs, rateAbs, Phi)
% [angRel, rateRel] = getAngRel(angAbs, rateAbs, Phi)
%
% This function converts from absolute coordinates to relative coordinates
%
% angAbs = [4,n]
%   (1) = phi0 = outer foot absolute angle
%   (2) = phi1 = inner foot absolute angle
%   (3) = th0 = outer foot absolute angle
%   (4) = th1 = inner foot absolute angle
%
% angRel = [4,n]
%   (1) = qr = -absolute angle of outer leg (from IMU)
%   (2) = qh = hip joint angle
%   (3) = q0 = outer ankle joint angle
%   (4) = q1 = inner ankle joint angle
%
% Phi = ankle orientation constant (~ 1.8 radians)
%

phi0 = angAbs(1,:);
phi1 = angAbs(2,:);
th0 = angAbs(3,:);
th1 = angAbs(4,:);

qr = -th0;
qh = th1-th0;
q0 = Phi - phi0 + th0;
q1 = Phi - phi1 + th1;

angRel = [qr;qh;q0;q1];

if nargout == 2
    dphi0 = rateAbs(1,:);
    dphi1 = rateAbs(2,:);
    dth0 = rateAbs(3,:);
    dth1 = rateAbs(4,:);
    
    dqr = -dth0;
    dqh = dth1-dth0;
    dq0 = -dphi0 + dth0;  % NOTE CONSTANT TERM DROPPED
    dq1 = -dphi1 + dth1;
    
    rateRel = [dqr;dqh;dq0;dq1];
end

end