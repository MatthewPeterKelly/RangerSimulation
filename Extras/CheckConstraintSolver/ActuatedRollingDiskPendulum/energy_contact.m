function [KE,PE] = energy_contact(z,dz,p)

x = z(1,:);
y = z(2,:);
q0 = z(3,:);
q1 = z(4,:);

dx = dz(1,:);
dy = dz(2,:);
dq0 = dz(3,:);
dq1 = dz(4,:);

[KE,PE] = autoGen_energy_contact(...
    x,y,q0,q1,...
    dx,dy,dq1,dq0,...
    p.m0,p.m1,p.g,p.r,p.a,p.b,p.I0,p.I1);

end