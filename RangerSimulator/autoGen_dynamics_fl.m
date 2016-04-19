function [M_fl,f_fl] = autoGen_dynamics_fl(th0,th1,dth0,dth1,u0,u1,uHip,fx,fy,g,c,m,I,Ifoot)
%AUTOGEN_DYNAMICS_FL
%    [M_FL,F_FL] = AUTOGEN_DYNAMICS_FL(TH0,TH1,DTH0,DTH1,U0,U1,UHIP,FX,FY,G,C,M,I,IFOOT)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    30-Jan-2016 13:22:41

t2 = cos(th0);
t3 = sin(th0);
t4 = cos(th1);
t5 = sin(th1);
t6 = c.^2;
t7 = -I-m.*t6;
M_fl = reshape([m.*-2.0,0.0,-c.*m.*t2,-c.*m.*t4,0.0,0.0,0.0,m.*-2.0,-c.*m.*t3,-c.*m.*t5,0.0,0.0,0.0,0.0,-Ifoot,0.0,-Ifoot,0.0,0.0,0.0,0.0,-Ifoot,0.0,-Ifoot,-c.*m.*t2,-c.*m.*t3,t7,0.0,0.0,0.0,-c.*m.*t4,-c.*m.*t5,0.0,t7,0.0,0.0],[6,6]);
if nargout > 1
    t8 = dth0.^2;
    t9 = dth1.^2;
    f_fl = [-fx-m.*(c.*t3.*t8.*(1.0./2.0)+c.*t5.*t9.*(1.0./2.0)).*2.0;-fy+g.*m.*2.0+c.*m.*t2.*t8+c.*m.*t4.*t9;uHip+c.*g.*m.*t3;-uHip+c.*g.*m.*t5;u0;u1];
end
