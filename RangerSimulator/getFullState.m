function x = getFullState(q,dq,p)
%  x = getFullState(q,dq,p)
%
% This function takes the joint-space configuration and computes the full
% state of the robot (includes hip position and velocity) by making the
% assumption that the lowest point on the robot is at the origin with zero
% velocity.
%
% INPUTS:
%   q = [qf1;qf2;ql1;ql2];
%   dq = [dqf1;dqf2;dql1;dql2];
%   p = parameter struct
%
% OUTPUTS:
%   x = full robot state:
%       1 = hip horizontal position
%       2 = hip vertical position
%       3 = stance foot one angle
%       4 = swing foot two angle
%       5 = stance leg one angle
%       6 = swing leg two angle
%       7 = hip horizontal velocity
%       8 = hip vertical velocity
%       9 = stance foot one angle rate
%       10 = swing foot two angle rate
%       11 = stance leg one angle rate
%       12 = swing leg two angle rate
%

z = [0;0;q];
dz = [0;0;dq];

% Figure out the contact angle for each foot:
[p1,p2] = kinematics(z,[],[],p);
qf1 = z(3);  qf2 = z(4);
[qc1, rc1] = getContactInfo(p1,qf1,p);
[qc2, rc2] = getContactInfo(p2,qf2,p);
qc = [qc1;qc2]; rc = [rc1;rc2];

% Determine the kinematics of each contact point
[c1,c2,dc1,dc2] = kinematicsContact(z,dz,qc,rc,p);

% Figure out which contact point to use:
if c1(2) < c2(2)
    % foot one is stance
    pos = -c1;
    vel = -dc1;
else
    % foot two is stance
    pos = -c2;
    vel = -dc2;
end

x = [pos;q;vel;dq];

end