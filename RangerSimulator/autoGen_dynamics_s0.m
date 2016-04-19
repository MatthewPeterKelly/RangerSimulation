function [M_s0,f_s0] = autoGen_dynamics_s0(th0,th1,phi0,dth0,dth1,dphi0,u0,u1,uHip,fx,fy,qc0,rc0,ddp0c_x,ddp0c_y,g,l,c,m,I,Ifoot,b)
%AUTOGEN_DYNAMICS_S0
%    [M_S0,F_S0] = AUTOGEN_DYNAMICS_S0(TH0,TH1,PHI0,DTH0,DTH1,DPHI0,U0,U1,UHIP,FX,FY,QC0,RC0,DDP0C_X,DDP0C_Y,G,L,C,M,I,IFOOT,B)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    30-Jan-2016 13:22:39

t2 = cos(th0);
t3 = sin(th0);
t4 = phi0+qc0;
t5 = cos(th1);
t6 = sin(th1);
t7 = c.^2;
t8 = -I-m.*t7;
t9 = cos(t4);
t10 = rc0.*t9;
t11 = sin(t4);
t12 = rc0.*t11;
t13 = l.*t2;
t14 = l.*t3;
M_s0 = reshape([m.*-2.0,0.0,-c.*m.*t2,-c.*m.*t5,0.0,0.0,-1.0,0.0,0.0,m.*-2.0,-c.*m.*t3,-c.*m.*t6,0.0,0.0,0.0,-1.0,0.0,0.0,-Ifoot,0.0,-Ifoot,0.0,-t10,-t12,0.0,0.0,0.0,-Ifoot,0.0,-Ifoot,0.0,0.0,-c.*m.*t2,-c.*m.*t3,t8,0.0,0.0,0.0,-t13,-t14,-c.*m.*t5,-c.*m.*t6,0.0,t8,0.0,0.0,0.0,0.0,1.0,0.0,t10+t13,0.0,t10,0.0,0.0,0.0,0.0,1.0,t12+t14,0.0,t12,0.0,0.0,0.0],[8,8]);
if nargout > 1
    t15 = dth0.^2;
    t16 = dth1.^2;
    t17 = b.*dphi0;
    t18 = dphi0.^2;
    f_s0 = [-fx-c.*m.*t3.*t15-c.*m.*t6.*t16;-fy+g.*m.*2.0+c.*m.*t2.*t15+c.*m.*t5.*t16;t17+uHip+c.*g.*m.*t3;-uHip+c.*g.*m.*t6;t17+u0;u1;-ddp0c_x-l.*t3.*t15-rc0.*t11.*t18;-ddp0c_y+l.*t2.*t15+rc0.*t9.*t18];
end
