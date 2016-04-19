function dZ = dynamics_minimal(Z,u,p)

q0 = Z(1,:);
q1 = Z(2,:);
dq0 = Z(3,:);
dq1 = Z(4,:);

[ddq0,ddq1] = autoGen_dynamics_minimal(...
    q0,q1,dq1,dq0,...
    u,...
    p.m0,p.m1,p.g,p.r,p.a,p.b,p.I0,p.I1,p.slope);

dZ = [dq0;dq1;ddq0;ddq1];

end