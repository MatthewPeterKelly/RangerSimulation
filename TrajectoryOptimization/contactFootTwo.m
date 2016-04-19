function cst = contactFootTwo(z,dz,p)
% cst = contactFootTwo(z,dz,p)
%
% This function computes the constraint that, if set to zero, implies that
% foot two is rolling on flat level ground.
%
% INPUTS:
%   z = [6 x n] configuration matrix
%   dz = [6 x n] configuration rate matrix
%   p = parameter struct:
%
% OUTPUTS
%   cst = [2 x n] constraint. if cst==0, then foot two is rolling
%

% x = z(1,:);
y = z(2,:);
% qf1 = z(3,:);
qf2 = z(4,:);
% ql1 = z(5,:);
ql2 = z(6,:);

dx = dz(1,:);
% dy = dz(2,:);
% dqf1 = dz(3,:);
dqf2 = dz(4,:);
% dql1 = dz(5,:);
dql2 = dz(6,:);

[cstHeight,cstRolling] = autoGen_cst_footTwo(...
   y,qf2,ql2,dx,dqf2,dql2,...
    p.d, p.l, p.r);

cst = [cstHeight; cstRolling];

end