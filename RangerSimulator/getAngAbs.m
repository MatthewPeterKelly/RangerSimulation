function [angAbs, rateAbs] = getAngAbs(angRel, rateRel, Phi)
% [angAbs, rateAbs] = getAngAbs(angRel, rateRel, Phi)
%
% This function converts from relative to absolute coordinates
%
% angRel = [4,n]
%   (1) = qr = -absolute angle of outer leg (from IMU)
%   (2) = qh = hip joint angle
%   (3) = q0 = outer ankle joint angle
%   (4) = q1 = inner ankle joint angle
%
% angAbs = [4,n]
%   (1) = phi0 = outer foot absolute angle
%   (2) = phi1 = inner foot absolute angle
%   (3) = th0 = outer foot absolute angle
%   (4) = th1 = inner foot absolute angle
%
% Phi = ankle orientation constant (~ 1.8 radians)
%

qr = angRel(1,:);
qh = angRel(2,:);
q0 = angRel(3,:);
q1 = angRel(4,:);

phi0 = Phi - qr - q0;
phi1 = Phi + qh - qr - q1;
th0 = -qr;
th1 = qh-qr;

angAbs = [phi0;phi1;th0;th1];

if nargout == 2
    dqr = rateRel(1,:);
    dqh = rateRel(2,:);
    dq0 = rateRel(3,:);
    dq1 = rateRel(4,:);
    
    dphi0 = -dqr - dq0;   %NOTE THAT CONSTANT TERM IS GONE
    dphi1 = dqh - dqr - dq1;
    dth0 = -dqr;
    dth1 = dqh-dqr;
    
    rateAbs = [dphi0;dphi1;dth0;dth1];
end

end