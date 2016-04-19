function ddz = dynamics_fl(z,dz,u,f,dyn)
% ddz = dynamics_fl(z,dz,u,f,qc,p)
%
% This function computes the dynamics of the Cornell Ranger, assuming that
% both feet are in the air. It also computes acceleration of the contact
% point on each foot.
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
%   ddz = [6 x 1] angle acceleration vector
%   ddc0 = [2 x 1] acceleration vector for outer foot contact
%   ddc1 = [2 x 1] acceleration vector for inner foot contact
% 
% NOTES:
% 
% 
% f = [4, 1] = contact force vector
% f = [f0; f1] = [outer; inner] contact forces
% 	(1) = horizontal component
% 	(2) = vertical component
% 
% Angle Convention:
% 	- All angles are in absolute reference frame
%     - zero corresponds to:
%     	(legs) = ankle joint directly below hip joint
%         (feet) = ankle joint directly below virtual foot center
%   
%   -->  ** Even though there are no contact forces, we still need
%   "contact" points for the angular momentum balance equations. For the
%   purposes of this function, any points will work, provided that they are
%   distinct from both each and the locations of the hip and ankle joints.
%


% x = z(1);
% y = z(2);
% phi0 = z(3);
% phi1 = z(4);
th0 = z(5);
th1 = z(6);

% dx = dz(1);
% dy = dz(2);
% dphi0 = dz(3);
% dphi1 = dz(4);
dth0 = dz(5);
dth1 = dz(6);

u0 = u(1);
u1 = u(2);
uHip = u(3);

fx = f(1);
fy = f(2);



%Call the flight dynamics
[M_fl,f_fl] = autoGen_dynamics_fl(...
    th0,th1,...
    dth0,dth1,...
    u0,u1,uHip,...
    fx, fy,...
    dyn.g, dyn.c, dyn.m, dyn.I, dyn.Ifoot);

ddz = M_fl\f_fl;

end