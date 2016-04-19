function [G,dG] = autoGen_centerOfMass(x,y,th0,th1,dx,dy,dth0,dth1,c)
%AUTOGEN_CENTEROFMASS
%    [G,DG] = AUTOGEN_CENTEROFMASS(X,Y,TH0,TH1,DX,DY,DTH0,DTH1,C)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    30-Jan-2016 13:22:38

t2 = cos(th0);
t3 = cos(th1);
t4 = sin(th0);
t5 = sin(th1);
G = [x+c.*t4.*(1.0./2.0)+c.*t5.*(1.0./2.0);y-c.*t2.*(1.0./2.0)-c.*t3.*(1.0./2.0)];
if nargout > 1
    dG = [dx+c.*dth0.*t2.*(1.0./2.0)+c.*dth1.*t3.*(1.0./2.0);dy+c.*dth0.*t4.*(1.0./2.0)+c.*dth1.*t5.*(1.0./2.0)];
end