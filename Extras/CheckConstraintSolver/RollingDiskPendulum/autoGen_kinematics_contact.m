function [p0,pStar,p1,dp0,dpStar,dp1] = autoGen_kinematics_contact(x,y,q0,q1,dx,dy,dq1,dq0,r,a,b)
%AUTOGEN_KINEMATICS_CONTACT
%    [P0,PSTAR,P1,DP0,DPSTAR,DP1] = AUTOGEN_KINEMATICS_CONTACT(X,Y,Q0,Q1,DX,DY,DQ1,DQ0,R,A,B)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    13-Aug-2015 13:11:36

p0 = [x;y];
if nargout > 1
    t2 = sin(q0);
    t3 = a.*t2;
    t4 = cos(q0);
    pStar = [t3+x;y-a.*t4];
end
if nargout > 2
    t5 = a.*dq0.*t4;
    t6 = cos(q1);
    t7 = a.*dq0.*t2;
    t8 = sin(q1);
    p1 = [t3+x+b.*t8;y-a.*t4-b.*t6];
end
if nargout > 3
    dp0 = [dx;dy];
end
if nargout > 4
    dpStar = [dx+t5;dy+t7];
end
if nargout > 5
    dp1 = [dx+t5+b.*dq1.*t6;dy+t7+b.*dq1.*t8];
end
