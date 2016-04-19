function [p0,p1,dp0,dp1,ddp0,ddp1] = kinematics(z,dz,ddz,dyn)
% [p0,p1,dp0,dp1,ddp0,ddp1] = kinematics(z,dz,ddz,dyn)
%
% This function computes the kinematics of the ankle joints of the robot
%
% INPUTS:
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
% OUTPUTS:
%   p0 = position of outer ankle joint
%   p1 = position of inner ankle joint
%   dp0 = position of outer ankle joint
%   dp1 = velocity of inner ankle joint
%   ddp0 = position of outer ankle joint
%   ddp1 = acceleration inner ankle joint
%
%   Kinematics:
%       1 = horizontal (x)
%       2 = vertical (y)
%   Angle Conventions:
%       - All angles are in absolute reference frame
%       - zero corresponds to:
%           (legs) = ankle joint directly below hip joint
%           (feet) = ankle joint directly below virtual foot center
%

x = z(1);
y = z(2);
% phi0 = z(3);
% phi1 = z(4);
th0 = z(5);
th1 = z(6);

if nargout > 4  %Then need velocities
dx = dz(1);
dy = dz(2);
% dphi0 = dz(3);
% dphi1 = dz(4);
dth0 = dz(5);
dth1 = dz(6);

end

if nargout > 8   %Then also need accelerations
ddx = ddz(1);
ddy = ddz(2);
% ddphi0 = ddz(3);
% ddphi1 = ddz(4);
ddth0 = ddz(5);
ddth1 = ddz(6);
end

if nargout <= 4 % Calculate positions onle
    [p0,p1] = autoGen_kinematics(...
        x,y,th0,th1,...
        [],[],[],[],...
        [],[],[],[],...
        dyn.l);
elseif nargout > 4 % calculate positions and velocities
    [p0,p1,dp0,dp1] = autoGen_kinematics(...
        x,y,th0,th1,...
        dx,dy,dth0,dth1,...
        [],[],[],[],...
        dyn.l);
    
elseif nargout > 8  % calculate position, velocity, and acceleration
    [p0,p1,dp0,dp1,ddp0,ddp1] = autoGen_kinematics(...
        x,y,th0,th1,...
        dx,dy,dth0,dth1,...
        ddx,ddy,ddth0,ddth1,...
        dyn.l);
    
end

end