function [pHip, p1, p2] = getJointPos_ds(z,p)
% [pHip, p1, p2] = getJointPos_ds(z,p)
%
% This function computes the position of the hip and ankle joints.
%
% INPUTS:
%   z = [4 x n] angle matrix
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
%   pHip = [2 x n] hip joint position
%   p1 = [2 x n] ankle joint one position
%   p2 = [2 x n] ankel joint two position
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

l = p.l;
d = p.d;
r = p.r;

% Figure out the reference foot spacing:
x0 = autoGen_getFootRefDist(qf1,qf2,ql1,ql2,l,d,r);

% Compute the kinematics:
[pHip,p1,p2] = autoGen_getJointPos(qf1,qf2,ql1,ql2,r,d,l,x0);

end
