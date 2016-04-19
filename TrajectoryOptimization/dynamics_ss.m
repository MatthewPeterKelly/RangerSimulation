function ddz = dynamics_ss(z,dz,u,p)
% ddz = dynamics_ss(z,dz,u,p)
%
% This function computes the single-stance dynamics of the Cornell Ranger
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
nState = 4;

qf1 = z(1,:);
% qf2 = z(2,:);
ql1 = z(3,:);
ql2 = z(4,:);

dqf1 = dz(1,:);
% dqf2 = dz(2,:);
dql1 = dz(3,:);
dql2 = dz(4,:);

u1 = u(1,:);
uHip = u(2,:);
u2 = u(3,:);

Ifoot = p.Ifoot*ones(1,nTime);  %Needed for vectorization

%%%% Vectorized call to dynamics
[Mnz,Mii,f] = autoGen_dynamics_ss(qf1,ql1,ql2,...%Angles
    dqf1,dql1,dql2,...  %Rates
    u1,u2,uHip,... %Torques
    p.g, p.l, p.d, p.r, p.c, p.m, p.I, Ifoot); %Parameters

%%%% Loop over the matrix inverse operation
ddz = zeros(size(z));
MM = zeros(nState,nState);
for i=1:nTime
    MM(Mii) = Mnz(:,i);
    ddz(:,i) =  MM\f(:,i);
end


end

