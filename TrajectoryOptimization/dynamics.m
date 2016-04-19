function ddz = dynamics(z,dz,u,f,p)
% ddz = dynamics(z,dz,u,f,p)
%
% This function computes the single-stance dynamics of the Cornell Ranger
%
% INPUTS:
%   z = [6 x n] configuration matrix
%   dz = [6 x n] configuration rate matrix
%   u = [3 x n] joint torque matrix
%   f = [4 x n] contact force matrix
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
%       1 = hip horizontal position
%       2 = hip vertical position
%       3 = stance foot one angle
%       4 = swing foot two angle
%       5 = stance leg one angle
%       6 = swing leg two angle
%   Torques:
%       1 = ankle joint one torque  (acting on foot)
%       2 = hip joint torque  (acting on swing leg)
%       3 = ankle joint torque  (acting on foot)
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


nTime = size(z,2);
nState = 6;

% x = z(1,:);
% y = z(2,:);
qf1 = z(3,:);
qf2 = z(4,:);
ql1 = z(5,:);
ql2 = z(6,:);

% dx = dz(1,:);
% dy = dz(2,:);
% dqf1 = dz(3,:);
% dqf2 = dz(4,:);
dql1 = dz(5,:);
dql2 = dz(6,:);

u1 = u(1,:);
uHip = u(2,:);
u2 = u(3,:);

F1x = f(1,:);
F1y = f(2,:);
F2x = f(3,:);
F2y = f(4,:);

%Needed for vectorization
ONES = ones(1,nTime);
d = p.d*ONES;
r = p.r*ONES;
g = p.g*ONES;
l = p.l*ONES;
c = p.c*ONES;
m = p.m*ONES;
I = p.I*ONES;
Ifoot = p.Ifoot*ONES;

%%%% Vectorized call to dynamics
[Mnz,Mii,FF] = autoGen_dynamics(...
    qf1,qf2,ql1,ql2,dql1,dql2,...
    u1,u2,uHip,F1x,F1y,F2x,F2y,...
    d,r,g,l,c,m,I,Ifoot);

%%%% Loop over the matrix inverse operation
ddz = zeros(size(z));
MM = zeros(nState,nState);
for i=1:nTime
    MM(Mii) = Mnz(:,i);
    ddz(:,i) =  MM\FF(:,i);
end


end

