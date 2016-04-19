function [KE,PE] = autoGen_energy_contact(x,y,q0,q1,dx,dy,dq1,dq0,m0,m1,g,r,a,b,I0,I1)
%AUTOGEN_ENERGY_CONTACT
%    [KE,PE] = AUTOGEN_ENERGY_CONTACT(X,Y,Q0,Q1,DX,DY,DQ1,DQ0,M0,M1,G,R,A,B,I0,I1)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    13-Aug-2015 13:11:37

t4 = cos(q0);
t5 = cos(q1);
t2 = dx+a.*dq0.*t4+b.*dq1.*t5;
t3 = dy+a.*dq0.*sin(q0)+b.*dq1.*sin(q1);
KE = I0.*dq0.^2.*(1.0./2.0)+I1.*dq1.^2.*(1.0./2.0)+m0.*(dx.^2+dy.^2).*(1.0./2.0)+m1.*(t2.^2+t3.^2).*(1.0./2.0);
if nargout > 1
    PE = g.*m0.*y-g.*m1.*(-y+a.*t4+b.*t5);
end
