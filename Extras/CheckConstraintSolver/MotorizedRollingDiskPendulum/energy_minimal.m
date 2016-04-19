function [KE,PE] = energy_minimal(Z,p)

q0 = Z(1,:);
q1 = Z(2,:);
dq0 = Z(3,:);
dq1 = Z(4,:);

[KE,PE] = autoGen_energy_minimal(...
    q0,q1,dq1,dq0,...
    p.m0,p.m1,p.g,p.r,p.a,p.b,p.I0,p.I1,p.slope);

end