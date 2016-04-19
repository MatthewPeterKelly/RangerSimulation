function [ddz, F1] = dynamics_s1(z,dz,u,f,qc,rc,ddc1,dyn)
% [ddz, F1] = dynamics_s1(z,dz,u,f,qc,rc,ddc1,dyn)
%
% This function computes the dynamics of the Cornell Ranger, assuming that
% the acceleration of the contact point on the inner foot is given.
%
% INPUTS:
% z = [6, 1] = full configuration vector
% dz = [6, 1] = full configuration rate vector
% 	(1) = hip horizontal position
% 	(2) = hip vertical position
% 	(3) = outer foot absolute angle
% 	(4) = inner foot absolute angle
% 	(5) = outer leg absolute angle
% 	(6) = inner leg absolute angle
%
% u = [3, 1] = motor torque vector
% 	(1) = outer ankle joint torque (+leg, -foot)
% 	(2) = inner ankle joint torque (+leg, -foot)
% 	(3) = hip torque (+inner, -outer)
%
% f = [2, 1] = disturbance force applied at hip:
%   (1) = fx = horizontal component
%   (2) = fy = vertical component
%
%   qc = [2 x 1] contact point angle relative to foot
%   rc = [2 x 1] contact point radius
%   ddc1 = [ 2 x 1] foot one contact point acceleration
%
% dyn = parameter struct:
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
% ddz = [6, 1] = full configuration acceleration vector
%   F1 = [2 x 1] contact forces on the inner feet
%   ddc0 = [2 x 1] acceleration of the contact point on the outer feet
%
%   Angle Conventions:
%       - All angles are in absolute reference frame
%       - zero corresponds to:
%           (legs) = ankle joint directly below hip joint
%           (feet) = ankle joint directly below virtual foot center
%



% x = z(1);
% y = z(2);
% phi0 = z(3);
phi1 = z(4);
th0 = z(5);
th1 = z(6);

% dx = dz(1);
% dy = dz(2);
% dphi0 = dz(3);
dphi1 = dz(4);
dth0 = dz(5);
dth1 = dz(6);

u0 = u(1);
u1 = u(2);
uHip = u(3);

fx = f(1);
fy = f(2);

% Contact points for each foot
% qc0 = qc(1);
qc1 = qc(2);
% rc0 = rc(1);
rc1 = rc(2);

% Acceleration of the virtual center of the foot
ddp1c_x = ddc1(1);
ddp1c_y = ddc1(2);

[M_s1,f_s1] = autoGen_dynamics_s1(...
    th0,th1,phi1,...
    dth0,dth1,dphi1,...
    u0,u1,uHip,...
    fx, fy,...
    qc1,rc1,ddp1c_x,ddp1c_y,...
    dyn.g,dyn.l,dyn.c,dyn.m,dyn.I,dyn.Ifoot,dyn.b);

vars = M_s1 \ f_s1;

ddz = vars(1:6);
F1 = vars(7:8);

end