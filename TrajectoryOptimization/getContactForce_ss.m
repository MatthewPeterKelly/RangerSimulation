function f1 = getContactForce_ss(z,dz,ddz,p)
% f1 = getContactForce_ss(z,dz,ddz,p)
%
% This function computes the contact forces acting on the stance foot
% during the single stance motion. 
%
% INPUTS:
%   z = [4 x n] angle matrix
%   dz = [4 x n] angle rate matrix
%   ddz = [4 x n] angle acceleration matrix
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
%   f1 = [2 x n] contact forces on foot one
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

ddqf1 = ddz(1,:);
% ddqf2 = ddz(2,:);
ddql1 = ddz(3,:);
ddql2 = ddz(4,:);

[f1x,f1y] = autoGen_contactForce_ss(...
    qf1,ql1,ql2,...
    dqf1,dql1,dql2,...
    ddqf1,ddql1,ddql2,...
    p.c, p.g, p.m, p.l, p.d, p.r);

f1 = [f1x;f1y];

end