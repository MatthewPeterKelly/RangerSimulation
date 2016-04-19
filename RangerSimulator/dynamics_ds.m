function [ddz, F0, F1] = dynamics_ds(z,dz,u,f,qc,rc,ddc,dyn)
% [ddz, F0, F1] = dynamics_ds(z,dz,u,f,qc,rc,ddc,dyn)
%
% This function computes the double stance dynamics of Cornell Ranger,
% assuming that the acceleration of the contact points is known.
%
% 
% z = [6, 1] = full configuration vector
% dz = [6, 1] = full configuration rate vector
% ddz = [6, 1] = full configuration acceleration vector
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
% dyn = parameter struct:
% 	.l = leg length
% 	.g = acceleration due to gravity
% 	.l = leg length (hip joint to foot joint)
% 	.d = distance between foot joint and virtual center of foot
% 	.r = radius of circular arc on foot
% 	.c = distance along leg from hip to CoM
% 	.w = distance off leg from main axis to CoM
% 	.m = mass of each leg
% 	.I = moment of inertia of the leg about its center of mass
% 	.Ifoot = moment of inertia of the foot about the foot joint
% 
% F = [4, 1] = contact force vector
% F = [F0; F1] = [outer; inner] contact forces
% 	(1) = horizontal component
% 	(2) = vertical component
% 
% Angle Convention:
% 	- All angles are in absolute reference frame
%     - zero corresponds to:
%     	(legs) = ankle joint directly below hip joint
%         (feet) = ankle joint directly below virtual foot center

nState = 6;
nContact = 4;

% x = z(1);
% y = z(2);
phi0 = z(3);
phi1 = z(4);
th0 = z(5);
th1 = z(6);

% dx = dz(1);
% dy = dz(2);
dphi0 = dz(3);
dphi1 = dz(4);
dth0 = dz(5);
dth1 = dz(6);

u0 = u(1);
u1 = u(2);
uHip = u(3);

fx = f(1);
fy = f(2);

% Contact points for each foot
qc0 = qc(1);
qc1 = qc(2);
rc0 = rc(1);
rc1 = rc(2);

% Acceleration of the contact points:
ddp0c_x = ddc(1);
ddp0c_y = ddc(2);
ddp1c_x = ddc(3);
ddp1c_y = ddc(4);

%%%% call to dynamics
[M_ds,f_ds] = autoGen_dynamics_ds(...
    phi0,phi1,th0,th1,...
    dphi0,dphi1,dth0,dth1,...
    u0,u1,uHip,...
    fx, fy,...
    qc0,qc1,rc1,rc0,...
    ddp1c_x,ddp1c_y,ddp0c_x,ddp0c_y,...
    dyn.g,dyn.l,dyn.c,dyn.m,dyn.I,dyn.Ifoot,dyn.b);
vars = M_ds\f_ds;

%%%% Unpack the result:
ddz = vars(1:nState);
F = vars( (nState+1) : (nState+nContact) );

F0 = F(1:2);
F1 = F(3:4);

end