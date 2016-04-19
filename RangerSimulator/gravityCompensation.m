function [u1,uHip,ddq1] = gravityCompensation(r,q,dq,dyn)
% [u1,uHip,ddq1] = gravityCompensation(r,q,dq,dyn)
%
% This function computes the torques required to cause zero acceleration in
% the stance foot and swing leg, given the current configuration.
%
% r = distance from instantaneous contact point on the stance foot to the
% stance ankle joint.
%
% q0 = virtual angle between the stance foot contact point and the stance
% angle joint. Positive j-axis corresponds to the zero angle.
%
% q = [q0;q1;q2];
%
% u1 = stance ankle torque (acting on the foot)
% uHip = hip torque acting on the swing leg.
% ddq1 = expected acceleration of the stance leg.

q0 = q(1);
q1 = q(2);
q2 = q(3);

dq0 = dq(1);
dq1 = dq(2);
dq2 = dq(3);

[MM,ff] = autoGen_gravityCompensation(q0,q1,q2,dq0,dq1,dq2,...
    dyn.l,dyn.c,r,dyn.m,dyn.g,dyn.I);

soln = MM\ff;   %Numerically solve linear system

u1 = soln(1);
ddq1 = soln(2);
uHip = soln(3);

end