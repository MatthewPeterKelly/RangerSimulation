function [p0c,p1c,dp0c,dp1c] = autoGen_kinematicsContact(x,y,phi0,phi1,th0,th1,dx,dy,dphi0,dphi1,dth0,dth1,qc0,qc1,rc0,rc1,l)
%AUTOGEN_KINEMATICSCONTACT
%    [P0C,P1C,DP0C,DP1C] = AUTOGEN_KINEMATICSCONTACT(X,Y,PHI0,PHI1,TH0,TH1,DX,DY,DPHI0,DPHI1,DTH0,DTH1,QC0,QC1,RC0,RC1,L)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    30-Jan-2016 13:22:37

t2 = phi0+qc0;
t3 = phi1+qc1;
t4 = cos(th0);
t5 = cos(t2);
t6 = sin(th0);
t7 = sin(t2);
p0c = [x+l.*t6+rc0.*t7;y-l.*t4-rc0.*t5];
if nargout > 1
    t8 = cos(th1);
    t9 = cos(t3);
    t10 = sin(th1);
    t11 = sin(t3);
    p1c = [x+l.*t10+rc1.*t11;y-l.*t8-rc1.*t9];
end
if nargout > 2
    dp0c = [dx+dth0.*l.*t4+dphi0.*rc0.*t5;dy+dth0.*l.*t6+dphi0.*rc0.*t7];
end
if nargout > 3
    dp1c = [dx+dth1.*l.*t8+dphi1.*rc1.*t9;dy+dth1.*l.*t10+dphi1.*rc1.*t11];
end