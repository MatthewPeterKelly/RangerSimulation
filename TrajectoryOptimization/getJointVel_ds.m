function [dpHip, dp1, dp2] = getJointVel(z,dz,p)
% [dpHip, dp1, dp2] = getJointVel(z,dz,p)
%
% This function computes the velocity of the hip and ankle joints.
%
% INPUTS:
%   z = [4 x n] angle matrix
%   dz = [4 x n] angle rate matrix
%   p = parameter struct:
%    .g = acceleration due to gravity
%    .l = leg length (hip joint to foot joint)
%    .d = distance between foot joint and virtual center of foot
%    .r = radius of circular arc on foot
%    .c = distance along leg from hip to CoM
%    .w = distance off leg from main axis to CoM
%    .m = mass of each leg
%    .I = moment of inertia of the leg about its center of mass
%    .Ifoot = moment of inertia of the foot about the foot joint
%
% OUTPUTS:
%   dpHip = [2 x n] hip joint velocity
%   dp1 = [2 x n] ankle joint one velocity
%   dp2 = [2 x n] ankel joint two velocity
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

qf1 = z(1,:);
qf2 = z(2,:);
ql1 = z(3,:);
ql2 = z(4,:);

dqf1 = dz(1,:);
dqf2 = dz(2,:);
dql1 = dz(3,:);
dql2 = dz(4,:);

l = p.l;
d = p.d;
r = p.r;

% Compute the kinematics:
[dpHip,dp1,dp2] = autoGen_getJointVel(...
    qf1,qf2,ql1,ql2,dqf1,dqf2,dql1,dql2,...
    r,d,l);

end
