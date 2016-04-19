function [ddz, F] = dynamics_contact(z,dz,P)

x = z(1,:);
y = z(2,:);
q = z(3,:);
dx = dz(1,:);
dy = dz(2,:);
dq = dz(3,:);

% Figure out what the contact point is doing:
th = -q + P.slope;
[pc,dpc] = autoGen_kinematics_contact(x,dx,y,dy,q,dq,th,P.r);

% Desired contact point:
c = pc(1)*[1; P.slope];  %Project the contact point to the ground
dc = [0;0];  %The contact point should be stationary

% Symplectic euler projection:
gain = sqrt(0.5);   %0.0 -> passive, 1.0 -> deadbeat, 0.5 -> critical damping
h = P.dt;
ddc = (gain*(c - pc) + h*(dc - dpc))/(h*h);  %Desired contact acceleration

% Call dynamics function
[ddx,ddy,ddq,Fx,Fy] = autoGen_dynamics_contact(q,dq,th,ddc(1,:),ddc(2,:),P.m,P.g,P.r,P.I);

% Pack up:
ddz = [ddx;ddy;ddq];
F = [Fx;Fy];

end