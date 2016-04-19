function [ddq0,ddq1] = autoGen_dynamics_minimal(q0,q1,dq1,dq0,u,m0,m1,g,r,a,b,I0,I1,slope)
%AUTOGEN_DYNAMICS_MINIMAL
%    [DDQ0,DDQ1] = AUTOGEN_DYNAMICS_MINIMAL(Q0,Q1,DQ1,DQ0,U,M0,M1,G,R,A,B,I0,I1,SLOPE)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    13-Aug-2015 14:46:51

t2 = b.^2;
t3 = m1.^2;
t4 = a.^2;
t5 = r.^2;
t6 = q1.*2.0;
t7 = q0-slope;
t8 = cos(t7);
t9 = q0-q1;
t10 = dq1.^2;
t11 = q1-slope;
t12 = sin(q0);
t13 = sin(slope);
t14 = q0.*2.0;
t15 = dq0.^2;
t16 = q0+slope-t6;
t17 = sin(t9);
t18 = sin(t7);
t19 = sin(t11);
t20 = -t6+t14;
t21 = I0.*I1.*2.0;
t22 = t2.*t3.*t4;
t23 = t2.*t3.*t5;
t24 = I1.*m1.*t4.*2.0;
t25 = I0.*m1.*t2.*2.0;
t26 = I1.*m0.*t5.*2.0;
t27 = I1.*m1.*t5.*2.0;
t28 = cos(t20);
t40 = slope.*2.0;
t29 = t6-t40;
t30 = cos(t29);
t31 = m0.*m1.*t2.*t5.*2.0;
t32 = cos(t16);
t33 = a.*r.*t2.*t3.*t32.*2.0;
t34 = t21+t22+t23+t24+t25+t26+t27+t31+t33-t2.*t3.*t4.*t28-t2.*t3.*t5.*t30-I1.*a.*m1.*r.*t8.*4.0-a.*r.*t2.*t3.*t8.*2.0;
t35 = 1.0./t34;
t36 = cos(t9);
t37 = cos(t11);
t38 = b.*m1.*r.*t37.*u.*2.0;
t39 = sin(q1);
t41 = sin(t20);
t42 = sin(t16);
ddq0 = -t35.*(t38-I1.*u.*2.0-m1.*t2.*u.*2.0+a.*g.*t2.*t3.*sin(q0-t6)+g.*r.*t2.*t3.*sin(-slope+t6)+I1.*a.*g.*m1.*t12.*2.0-I1.*g.*m0.*r.*t13.*2.0-I1.*g.*m1.*r.*t13.*2.0-a.*b.*m1.*t36.*u.*2.0+a.*g.*t2.*t3.*t12-g.*r.*t2.*t3.*t13+t2.*t3.*t4.*t15.*t41+a.*r.*t2.*t3.*t15.*t18-a.*r.*t2.*t3.*t15.*t42+b.*r.*t2.*t3.*t10.*t19.*2.0+I1.*a.*b.*m1.*t10.*t17.*2.0+I1.*a.*m1.*r.*t15.*t18.*2.0+I1.*b.*m1.*r.*t10.*t19.*2.0+a.*b.*t2.*t3.*t10.*t17.*2.0-g.*m0.*m1.*r.*t2.*t13.*2.0);
if nargout > 1
    t43 = q1-t40;
    t44 = sin(t43);
    t45 = q0-q1+slope;
    t46 = sin(t45);
    t47 = q0-q1-slope;
    t48 = sin(t47);
    ddq1 = -t35.*(-t38+I0.*u.*2.0+m0.*t5.*u.*2.0+m1.*t4.*u.*2.0+m1.*t5.*u.*2.0+t2.*t3.*t5.*t10.*sin(t29)-b.*g.*t3.*t4.*sin(-q1+t14)+I0.*b.*g.*m1.*t39.*2.0+a.*b.*m1.*t36.*u.*2.0+b.*g.*t3.*t4.*t39+b.*g.*t3.*t5.*t39+b.*g.*t3.*t5.*t44-a.*m1.*r.*t8.*u.*4.0-t2.*t3.*t4.*t10.*t41+a.*r.*t2.*t3.*t10.*t42.*2.0-b.*r.*t3.*t4.*t15.*t19.*3.0+a.*b.*t3.*t5.*t15.*sin(q0+q1-t40)+b.*r.*t3.*t4.*t15.*sin(-q1-slope+t14)-I0.*a.*b.*m1.*t15.*t17.*2.0+a.*b.*g.*r.*t3.*t46.*2.0+a.*b.*g.*r.*t3.*t48+b.*g.*m0.*m1.*t5.*t39+b.*g.*m0.*m1.*t5.*t44-a.*b.*t3.*t4.*t15.*t17.*2.0-a.*b.*t3.*t5.*t15.*t17-a.*b.*g.*r.*t3.*sin(q0+q1-slope)+a.*b.*g.*m0.*m1.*r.*t46-a.*b.*g.*m0.*m1.*r.*t48-a.*b.*m0.*m1.*t5.*t15.*t17.*2.0);
end
