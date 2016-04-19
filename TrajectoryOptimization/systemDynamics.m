function [ddz, u] = systemDynamics(z,dz,dyn,ref)
% [ddz, u] = systemDynamics(z,dz,dyn,ref)
%
% This function computes the single-stance dynamics of the Cornell Ranger
%
% INPUTS:
%   z = [4, 1] angle vector
%   dz = [4, 1] angle rate vector
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
%   ddz = [4, 1] angle acceleration vector
%   u = [3, 1] vector of control torques
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


qf1 = z(1,:);
% qf2 = z(2,:);
ql1 = z(3,:);
ql2 = z(4,:);

dqf1 = dz(1,:);
% dqf2 = dz(2,:);
dql1 = dz(3,:);
dql2 = dz(4,:);

% Compute reference trajectories:
p = ref.c*z;
hRef = ppval(ref.pp.h, p);
dhRef = ppval(ref.pp.dh, p);
dhRefdt = ppval(ref.pp.dhdt , p);
ddhRef = ppval(ref.pp.ddh, p);
h = ref.H*z;
dhdt = ref.H*dz;

% Linear controller:
kp = ref.wn^2;
kd = 2*ref.xi*ref.wn;
v = kp*(hRef-h) + kd*(dhRefdt-dhdt);  

% D*ddq + G = B*u
[D,G,B] = autoGen_dyn_ss_mat(...
    qf1,ql1,ql2,dqf1,dql1,dql2,...
     dyn.g, dyn.l, dyn.d, dyn.r, dyn.c, dyn.m, dyn.I, dyn.Ifoot);

u = controlMap(dz, v, dhRef, ddhRef, ref.c, ref.H, D, B, G);
ddz = D\(B*u - G);

end