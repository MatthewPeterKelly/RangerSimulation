function [p2c, dp2c] = contactKinematics(z,dz,p)
% [p2c, dp2c] = contactKinematics(z,dz,p)
%
% This function computes the position and velocity of the contact point on
% the swing foot
%
% INPUTS:
%   z = [4 x n] angle matrix
%   dz = [4 x n] angle rate matrix
%   p = parameter struct:
%    .l = leg length (hip joint to foot joint)
%    .d = distance between foot joint and virtual center of foot
%    .r = radius of circular arc on foot
%
% OUTPUTS:
%   p1 = [2 x n] ankle one joint position [x;y]
%   pHip = [2 x n] hip joint position [x;y]
%   p2 = [2 x n] ankle two joint position [x;y]
%   dp1 = [2 x n] ankle one joint velocity [x;y]
%   dpHip = [2 x n] hip joint velocity [x;y]
%   dp2 = [2 x n] ankle two joint velocity [x;y]
%
% NOTES:
%   States:
%       1 = stance foot one angle
%       2 = swing foot two angle
%       3 = stance leg one angle
%       4 = swing leg two angle
%   Angle Conventions:
%       - All angles are in absolute reference frame
%       - zero corresponds to:
%           (legs) = ankle joint directly below hip joint
%           (feet) = ankle joint directly below virtual foot center
%


qf1 = z(1,:);
qf2 = z(2,:);
ql1 = z(3,:);
ql2 = z(4,:);

if nargout ==1  %Position only

    p2c = autoGen_contactKinematics(...
        qf1,qf2,ql1,ql2,...
        [],[],[],[],...
        p.l,p.d,p.r);

else %Position and Velocity

    dqf1 = dz(1,:);
    dqf2 = dz(2,:);
    dql1 = dz(3,:);
    dql2 = dz(4,:);
    
    [p2c,dp2c] = autoGen_contactKinematics(...
        qf1,qf2,ql1,ql2,...
        dqf1,dqf2,dql1,dql2,...
        p.l,p.d,p.r);

end

end