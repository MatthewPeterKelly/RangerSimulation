function [p1,pHip,p2,dp1,dpHip,dp2] = autoGen_kinematics_ss(qf1,ql1,ql2,dqf1,dql1,dql2,l,d,r)
%AUTOGEN_KINEMATICS_SS
%    [P1,PHIP,P2,DP1,DPHIP,DP2] = AUTOGEN_KINEMATICS_SS(QF1,QL1,QL2,DQF1,DQL1,DQL2,L,D,R)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    13-Jun-2015 09:38:55

t2 = sin(qf1);
t3 = d.*t2;
t4 = cos(qf1);
t5 = sin(ql1);
t6 = cos(ql1);
t7 = l.*t6;
t9 = d.*t4;
t8 = r-t9;
p1 = [t3-qf1.*r;t8];
if nargout > 1
    pHip = [t3-l.*t5-qf1.*r;r+t7-d.*t4];
end
if nargout > 2
    t10 = d.*dqf1.*t2;
    t11 = cos(ql2);
    t12 = sin(ql2);
    p2 = [t3-l.*t5+l.*t12-qf1.*r;r+t7-d.*t4-l.*t11];
end
if nargout > 3
    dp1 = [-dqf1.*t8;t10];
end
if nargout > 4
    dpHip = [-dqf1.*t8-dql1.*l.*t6;t10-dql1.*l.*t5];
end
if nargout > 5
    dp2 = [-dqf1.*t8-dql1.*l.*t6+dql2.*l.*t11;t10-dql1.*l.*t5+dql2.*l.*t12];
end
