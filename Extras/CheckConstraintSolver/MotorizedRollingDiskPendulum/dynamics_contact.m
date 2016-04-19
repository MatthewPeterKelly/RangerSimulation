function[ddz,F] = dynamics_contact(z,dz,u,p)

x = z(1,:);
y = z(2,:);
q0 = z(3,:);
q1 = z(4,:);

dx = dz(1,:);
dy = dz(2,:);
dq0 = dz(3,:);
dq1 = dz(4,:);

% Figure out what the contact point is doing:
th = -q0 + p.slope;
[pc,dpc] = autoGen_contactPoint(x,y,q0,dx,dy,dq0,th,p.r);

% Desired contact point:
c = pc(1)*[1; p.slope];  %Project the contact point to the ground
dc = [0;0];  %The contact point should be stationary

% Constraint solver:
h = p.dt;
wn = 1/(2*h);
xi = 1.0;
kp = wn*wn;
kd = 2*xi*wn;
ddc = kp*(c - pc) + kd*(dc - dpc);  %Desired contact acceleration

% Call dynamics function
[AA,bb] = autoGen_dynamics_contact(...
    q0,q1,dq1,dq0,u,ddc(1),ddc(2),th,...
    p.m0,p.m1,p.g,p.r,p.a,p.b,p.I0,p.I1);
soln = AA\bb;

% Pack up:
ddz = soln(1:4);
F = soln(5:6);

end