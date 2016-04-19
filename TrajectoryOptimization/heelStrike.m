function [za, dza] = heelStrike(z,dz,p)
% [za, dza] = heelStrike(z,dz,p)
%
% Computes the heel-strike map (including switching feet) for Cornell
% Ranger, mapping from Single-Stance-One to Single-Stance-Two.
%
% INPUTS:
%     z = [4 x 1] angle vector
%     dz = [4 x 1] angle rate vector
%     p = parameter struct:
%      .l = leg length (hip joint to foot joint)
%      .d = distance between foot joint and virtual center of foot
%      .r = radius of circular arc on foot
%      .c = distance along leg from hip to CoM
%      .m = mass of each leg
%      .I = moment of inertia of the leg about its center of mass
%      .Ifoot = moment of inertia of the foot about the foot joint
%  
%   OUTPUTS:
%     za =  [4 x 1] angle vector after heel-strike
%     dza = [4 x 1] angle rate vector after heel-strike
%  
%   NOTES:
%     States:
%         1 = stance foot one angle
%         2 = swing foot two angle
%         3 = stance leg one angle
%         4 = swing leg two angle
%     Angle Conventions:
%         - All angles are in absolute reference frame
%         - zero corresponds to:
%             (legs) = ankle joint directly below hip joint
%             (feet) = ankle joint directly below virtual foot center
%     Map is from single stance to single stance 
%

[~,~,~,~,dpHip] = kinematics_ss(z,dz,p);

%Switch legs and feet
za = [z(2); z(1); z(4); z(3)];

qf1 = z(2);
ql1 = z(4);
ql2 = z(3);
dxb = dpHip(1);
dyb = dpHip(2);
dqf1b = dz(1);
dqf2b = dz(2);
dql1b = dz(3);
dql2b = dz(4);

[M,f] = autoGen_heelStrike(...
    qf1,ql1,ql2,...
    dxb,dyb,dqf1b,dqf2b,dql1b,dql2b,...
    p.I, p.Ifoot, p.c, p.d, p.l, p.m, p.r);
vars = M\f;
dza = vars(3:6);  

end