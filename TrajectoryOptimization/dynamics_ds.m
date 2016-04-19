function [ddz, f] = dynamics_ds(z,dz,u,p)
% [ddz, f] = dynamics_ds(z,dz,u,p)
%
% This function computes the double-stance dynamics of the Cornell Ranger
%
% INPUTS:
%   z = [4 x n] angle matrix
%   dz = [4 x n] angle rate matrix
%   u = [3 x n] joint torque matrix
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
%   ddz = [4 x n] angle acceleration matrix
% 
% NOTES:
%   States:
%       1 = stance foot one angle
%       2 = swing foot two angle
%       3 = stance leg one angle
%       4 = swing leg two angle
%   Torques:
%       1 = ankle joint one torque  (acting on foot)
%       2 = hip joint torque  (acting on swing leg)
%       3 = ankle joint torque  (acting on foot)
%   Angle Conventions:
%       - All angles are in absolute reference frame
%       - zero corresponds to:
%           (legs) = ankle joint directly below hip joint
%           (feet) = ankle joint directly below virtual foot center
%

nTime = size(z,2);
nVars = 4+4;

qf1 = z(1,:);
qf2 = z(2,:);
ql1 = z(3,:);
ql2 = z(4,:);

dqf1 = dz(1,:);
dqf2 = dz(2,:);
dql1 = dz(3,:);
dql2 = dz(4,:);

u1 = u(1,:);
uHip = u(2,:);
u2 = u(3,:);

g = p.g;
l = p.l;
d = p.d;
r = p.r;
c = p.c;
m = p.m;
I = p.I;
Ifoot = p.Ifoot;
one = ones(1,nTime);

% Project the joint angles to the constraint manifold:
zStar = autoGen_projectConstraints(qf1,qf2,ql1,ql2,l,d);
qf1 = zStar(1,:);
qf2 = zStar(2,:);
ql1 = zStar(3,:);
ql2 = zStar(4,:);

% Figure out the reference foot spacing:
x0 = autoGen_getFootRefDist(qf1,qf2,ql1,ql2,l,d,r);

% Vectorized call to the dynamics:
[Mnz,Mii,f] = autoGen_dynamics_ds(...
    qf1,qf2,ql1,ql2,dqf1,dqf2,dql1,dql2,...
    u1,u2,uHip,...
    g,l,d,r,c,m,I,Ifoot,x0,one);

%%%% Loop over the matrix inverse operation
vars = zeros(nVars,nTime);
MM = zeros(nVars,nVars);
for i=1:nTime
    MM(Mii) = Mnz(:,i);
    vars(:,i) =  MM\f(:,i);
end

ddz = vars(1:4,:);
f = vars(5:8,:);

end