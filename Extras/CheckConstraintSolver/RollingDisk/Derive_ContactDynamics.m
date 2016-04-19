% Derive Contact-Based dynamics for the rolling disk
%
%

clear; clc;

syms q dq ddq 'real' %Angle state 
syms x dx ddx 'real'  %Horizontal state
syms y dy ddy 'real'  %Vertical state
syms Fx Fy 'real' % Contact forces
syms slope m g r I 'real' % Parameters

% Angle from the center of the disk to the contact point, 
% in the frame of the disk, measured from negative y axis
syms th 'real' 

syms ddpc_x ddpc_y 'real' % Contact constraint:

i = sym([1;0]);
j = sym([0;1]);

z = [x;y;q;dx;dy;dq];
dz = [dx;dy;dq;ddx;ddy;ddq];

%%%% Kinematics:

e = cos(q+th)*(-j) + sin(q+th)*i; %Unit vector from center to contact

p = x*i + y*j;
dp = jacobian(p,z)*dz;  %Velocity
ddp = jacobian(dp,z)*dz;  %acceleration

pc = p + r*e;   %Position of contact point
dpc = jacobian(pc,z)*dz;  %Velocity
ddpc = jacobian(dpc,z)*dz;  %acceleration

%%%% Dynamics:

% Define a function for doing '2d' cross product: dot(cross(a,b), k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

F = Fx*i + Fy*j;
sumForces = F + m*g*(-j);
sumTorques = cross2d(r*e,F);  

eqns = [...
    sumForces - m*ddp;
    sumTorques - I*ddq;
    ddpc - [ddpc_x; ddpc_y]];

[AA,bb] = equationsToMatrix(eqns,[ddx;ddy;ddq;Fx;Fy]);
soln = simplify(AA\bb);

%%%% Write out solution:
matlabFunction(soln(1), soln(2), soln(3), soln(4), soln(5),...
    'file','autoGen_dynamics_contact.m',...
    'vars',{q,dq,th,ddpc_x, ddpc_y, m,g,r,I},...
    'output',{'ddx','ddy','ddq','Fx','Fy'});

%%%% Write out contact mechanics:
matlabFunction(pc,dpc,...
    'file','autoGen_kinematics_contact.m',...
    'vars',{x,dx,y,dy,q,dq,th,r},...
    'output',{'pc','dpc'});



