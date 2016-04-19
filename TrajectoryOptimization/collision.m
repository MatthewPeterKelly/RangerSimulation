function dza = collision(z,dzb,J,p)
% dzAfter = collision(z,dzBefore,J,p)
%
% This function computes the velocity of robot after a collision impulse is
% applied to the bottom of each foot.
%
% INPUTS:
%   z = [6 x n] configuration vector
%   dzb = [6 x n] configuration rate vector before collision
%   J = [4 x n] contact impulse vector
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
%   dza = [6 x n] configuration rate vector after collision
% 
% NOTES:
%   States:
%       1 = hip horizontal position
%       2 = hip vertical position
%       3 = stance foot one angle
%       4 = swing foot two angle
%       5 = stance leg one angle
%       6 = swing leg two angle
%   Contact Forces:
%       1 = foot one horizontal
%       2 = foot one vertical
%       3 = foot one horizontal
%       4 = foot one vertical
%   Angle Conventions:
%       - All angles are in absolute reference frame
%       - zero corresponds to:
%           (legs) = ankle joint directly below hip joint
%           (feet) = ankle joint directly below virtual foot center
% 
%
%

% x = z(1);
% y = z(2);
qf1 = z(3);
qf2 = z(4);
ql1 = z(5);
ql2 = z(6);

% dxb = dz(1);
% dyb = dz(2);
dqf1b = dzb(3);
dqf2b = dzb(4);
dql1b = dzb(5);
dql2b = dzb(6);

J1x = J(1);
J1y = J(2);
J2x = J(3);
J2y = J(4);


[M,f] = autoGen_collision(...
    qf1,qf2,ql1,ql2,...
    dxb,dyb,dqf1b,dqf2b,dql1b,dql2b,...
    J1x,J1y,J2x,J2y,...
    p.d, p.r, p.l, p.c, p.m, p.I, p.Ifoot);

dza = M\f;  %Velocity after collision

end