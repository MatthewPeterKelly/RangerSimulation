% Derive Minimal coordinates for a disk rolling down an incline plane
%
%

clc; clear;

syms q dq ddq 'real' %Minimal States
syms Fx Fy 'real' % Contact forces
syms slope m g r I 'real' % Parameters

i = sym([1;0]);
j = sym([0;1]);

%%%% Kinematics:

t = cos(slope)*i + sin(slope)*j;   % Unit vector along slope
n = -sin(slope)*i + cos(slope)*j;  % Uni vector normal to slope

z = [q;dq];
dz = [dq;ddq];

p = -r*q*t + r*n;   % Position of the center of the wheel
dp = jacobian(p,z)*dz;  %Velocity
ddp = jacobian(dp,z)*dz;  %acceleration


%%%% Dynamics:

% Define a function for doing '2d' cross product: dot(cross(a,b), k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

F = Fx*i + Fy*j;
sumForces = F + m*g*(-j);
sumTorques = cross2d(-r*n,F);  

eqns = [...
    sumForces - m*ddp;
    sumTorques - I*ddq];

[AA,bb] = equationsToMatrix(eqns,[ddq; Fx; Fy]);
soln = simplify(AA\bb);


%%%% Write out solution:
matlabFunction(soln(1), soln(2), soln(3),...
    'file','autoGen_dynamics_minimal.m',...
    'vars',{m,g,r,I,slope},...
    'output',{'ddq','Fx','Fy'});

%%%% Write out kinematics:
matlabFunction(p,dp,ddp,...
    'file','autoGen_kinematics.m',...
    'vars',{q,dq,ddq,r,slope},...
    'output',{'p','dp','ddp'});


