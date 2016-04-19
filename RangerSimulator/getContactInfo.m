function [qc, rc] = getContactInfo(pAnkle,qf,param)
%
% This function computes the polar coordinates of the contact point, in the
% frame of the foot.
%
% INPUTS:
%   p = position of the ankle joint
%   qf = absolute angle of the foot
%   param = parameter struct
%
% OUTPUTS:
%   qc = contact angle, in the frame of the foot
%   rc = radius from ankle joint to contact point, along qc
%

% HARD CODED PARAMETERS:
maxIter = 16;
tol = 1e-6;

% Run optimization to find best point:
[qSoln, ~, rc] = findContactPoint(-pi,pi,maxIter,tol,pAnkle,qf,param.ground);

qc = qSoln-qf;   %Contact angle in foot frame

end

