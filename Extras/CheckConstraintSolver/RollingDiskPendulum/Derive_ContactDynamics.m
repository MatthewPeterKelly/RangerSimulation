% Derive Contact-Based dynamics for the rolling disk
%
%

clear; clc;

syms x dx ddx 'real'  %Horizontal state - center of disk
syms y dy ddy 'real'  %Vertical state - center of disk
syms q0 dq0 ddq0 'real' %Minimal States of the disk
syms q1 dq1 ddq1 'real' %Minimal States of the pendulum
syms Fx Fy 'real' % Contact forces
syms m0 m1 g r a b I0 I1 'real' % Parameters
% r = radius of the disk
% a = distance from disk center to pin joint
% b = distance from pin joint to CoM of pendulum
%


% Angle from the center of the disk to the contact point, 
% in the frame of the disk, measured from negative y axis
syms th 'real' 

syms ddpc_x ddpc_y 'real' % Contact constraint:

i = sym([1;0]);
j = sym([0;1]);

%%%% Kinematics:

e0 = cos(q0)*(-j) + sin(q0)*(i);  %Unit vector from disk center to pin joint
e1 = cos(q1)*(-j) + sin(q1)*(i);  %Unit vector from pin joint to tip of pendulum

ec = cos(q0+th)*(-j) + sin(q0+th)*i; %Unit vector from center to contact

z = [x;y;q0;q1;dx;dy;dq0;dq1];
dz = [dx;dy;dq0;dq1;ddx;ddy;ddq0;ddq1];

p0 = x*i + y*j;   % Position of the center of the disk
dp0 = jacobian(p0,z)*dz;  %Velocity
ddp0 = jacobian(dp0,z)*dz;  %acceleration

pc = p0 + r*ec;  % Position of the contact point
dpc = jacobian(pc,z)*dz;  %Velocity
ddpc = jacobian(dpc,z)*dz;  %acceleration

pStar = p0 + a*e0;  %Position of the pin joint 
dpStar = jacobian(pStar,z)*dz;  %Velocity
ddpStar = jacobian(dpStar,z)*dz;  %acceleration

p1 = pStar + b*e1;  %Position of the pendulum CoM 
dp1 = jacobian(p1,z)*dz;  %Velocity
ddp1 = jacobian(dp1,z)*dz;  %acceleration



%%%% Dynamics:

% Define a function for doing '2d' cross product: dot(cross(a,b), k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

% Sum of forces and torques on entire system:
F = Fx*i + Fy*j;  %Contact forces
sumForces0 = F + m0*g*(-j) + m1*g*(-j);
sumAccel0 = m0*ddp0 + m1*ddp1;
sumTorques0 = cross2d(pc-p0,F) + cross2d(p1-p0,m1*g*(-j));  
sumInertial0 = I0*ddq0 + I1*ddq1 + cross2d(p1-p0,m1*ddp1);

% Sum of torques on pendulum about pin joint
sumTorques1 = cross2d(p1-pStar,m1*g*(-j));
sumInertial1 = I1*ddq1 + cross2d(p1-pStar,m1*ddp1);

% Collect equations:
eqns = [...
    sumForces0 - sumAccel0;
    sumTorques0 - sumInertial0;
    sumTorques1 - sumInertial1;
    ddpc(1) - ddpc_x;
    ddpc(2) - ddpc_y];
vars = [ddx;ddy;ddq0;ddq1;Fx;Fy];

[AA,bb] = equationsToMatrix(eqns,vars);

%%%% Energy:
KE = 0.5*I0*dq0*dq0 + 0.5*I1*dq1*dq1 + 0.5*m0*dot(dp0,dp0) + 0.5*m1*dot(dp1,dp1);
PE = m0*g*p0(2) + m1*g*p1(2);


%%%% Write out solution:
matlabFunction(AA,bb,...
    'file','autoGen_dynamics_contact.m',...
    'vars',{q0,q1,dq1,dq0,ddpc_x,ddpc_y,th,m0, m1, g, r, a, b, I0, I1},...
    'output',{'AA','bb'});

%%%% Get the contact point:
matlabFunction(pc,dpc,...
    'file','autoGen_contactPoint.m',...
    'vars',{x,y,q0,dx,dy,dq0, th, r},...
    'output',{'pc','dpc'});


%%%% Write out kinematics:
matlabFunction(p0,pStar,p1,dp0,dpStar,dp1,...
    'file','autoGen_kinematics_contact.m',...
    'vars',{x,y,q0,q1,dx,dy,dq1,dq0, r, a, b},...
    'output',{'p0','pStar','p1','dp0','dpStar','dp1'});


%%%% Write out energy:
matlabFunction(KE, PE,...
    'file','autoGen_energy_contact.m',...
    'vars',{x,y,q0,q1,dx,dy,dq1,dq0,m0, m1, g, r, a, b, I0, I1},...
    'output',{'KE','PE'});


