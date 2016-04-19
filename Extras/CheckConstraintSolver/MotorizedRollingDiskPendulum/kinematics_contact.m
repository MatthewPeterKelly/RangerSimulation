function [p0,pStar,p1,dp0,dpStar,dp1] = kinematics_contact(z,dz,p)

x = z(1,:);
y = z(2,:);
q0 = z(3,:);
q1 = z(4,:);

dx = dz(1,:);
dy = dz(2,:);
dq0 = dz(3,:);
dq1 = dz(4,:);

[p0,pStar,p1,dp0,dpStar,dp1] = autoGen_kinematics_contact(...
    x,y,q0,q1,...
    dx,dy,dq1,dq0,...
    p.r,p.a,p.b);

end