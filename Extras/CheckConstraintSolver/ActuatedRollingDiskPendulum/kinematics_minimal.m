function [p0,pStar,p1,dp0,dpStar,dp1] = kinematics_minimal(z,p)

q0 = z(1,:);
q1 = z(2,:);
dq1 = z(3,:);
dq0 = z(4,:);

[p0,pStar,p1,dp0,dpStar,dp1] = autoGen_kinematics_minimal(...
    q0,q1,dq1,dq0,...
    p.r, p.a, p.b, p.slope);

end

