function dz = simpleDynamics(z,xi,wn)

% The dynamics of a simple point mass, that is constrainted to be at the
% origin with zero position and velocity. The constraint is approximated by
% a critically damped pd controller.


x = z(1,:);
v = z(2,:);

% Constraint solver:  
kp = wn*wn;
kd = 2*xi*wn;
a = -kp*x + -kd*v;

dz = [v;a];

end