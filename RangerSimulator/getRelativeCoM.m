function x = getRelativeCoM(z, isStanceOuter, dyn)
% x = getRelativeCoM(z, isStanceOuter, dyn)
%
% This function computes the horizontal position of the center of mass,
% with respect to the location of the specified ankle joint
%

x = z(1);
y = z(2);
% phi0 = z(3);
% phi1 = z(4);
th0 = z(5);
th1 = z(6);

pCoM = autoGen_centerOfMass(x,y,th0,th1,[],[],[],[],dyn.c);
[p0,p1] = kinematics(z,[],[],dyn);

if isStanceOuter
    x = pCoM - p0(1);   % Outer legs are in stance
else
    x = pCoM - p1(1);   % Inner legs are in stance
end

end