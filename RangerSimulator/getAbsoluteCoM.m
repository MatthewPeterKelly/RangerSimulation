function [G, dG] = getAbsoluteCoM(state, dyn)
% [G, dG] = getAbsoluteCoM(x, dyn)
%
% This function computes the position and velocity of the center of mass
%

x = state(1);
y = state(2);
% phi0 = z(3);
% phi1 = z(4);
th0 = state(5);
th1 = state(6);



if nargout == 1
    G = autoGen_centerOfMass(x,y,th0,th1,[],[],[],[],dyn.c);
else
    dx = state(7);
    dy = state(8);
    % dphi0 = z(9);
    % dphi1 = z(10);
    dth0 = state(11);
    dth1 = state(12);
    
    [G, dG] = autoGen_centerOfMass(x,y,th0,th1,dx,dy,dth0,dth1,dyn.c);
    
end

end