function [pHip,p1,p2,dpHip,dp1,dp2] = kinematics_ss(z,dz,p)
%[pHip,p1,p2,dpHip,dp1,dp2] = kinematics_ss(z,dz,p)
%
% This function computes the joint positions and velocities for the
% single-stance phase of motion for the Cornell Ranger
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
% qf2 = z(2,:);
ql1 = z(3,:);
ql2 = z(4,:);

dqf1 = dz(1,:);
% dqf2 = dz(2,:);
dql1 = dz(3,:);
dql2 = dz(4,:);

[p1,pHip,p2,dp1,dpHip,dp2] = autoGen_kinematics_ss(qf1,ql1,ql2,...%Angles
    dqf1,dql1,dql2,...  %Rates
    p.l, p.d, p.r); %Parameters


end

