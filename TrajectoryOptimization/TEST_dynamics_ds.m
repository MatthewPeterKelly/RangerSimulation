% TEST_dynamics_ds.m
%
% This script is used to test the double stance dynamics equations


%%%% Parameters for Dynamics %%%%
p.g = 9.81; % acceleration due to gravity
p.l = 0.96; % leg length (hip joint to foot joint)
p.d = 0.14; % distance between foot joint and virtual center of foot
p.r = 0.2; % radius of circular arc on foot
p.c = 0.15; % distance along leg from hip to CoM
p.m = 4.96; % mass of each leg
p.I = 0.45; % moment of inertia of the leg about its center of mass
p.Ifoot = 0.05; % moment of inertia of the foot about the foot joint

p.angleToe = 30*(pi/180);
p.angleHeel = -5*(pi/180);


%%%% Set up initial conditions:
z0 = [...  %Change this line!
	0;    %Foot one absolute angle
 	0.3;   %Foot two absolute angle
	0.22;  %Leg one absolute angle
	-0.2];    %leg two absolute angle

%Project initial configuration to constraint manifold:
% There are four rigid bodies, but only three degrees of freedom because of
% the closed kinematic chain. This short block of code ensures that there
% is no constraint violation in the initial state.
for iter = 1:25
   zStar = autoGen_projectConstraints(z0(1),z0(2),z0(3),z0(4),p.l,p.d); 
   if norm(zStar-z0) < 1e-10  %Then on manifold
       break;
   else  %Keep iterating
       z0 = zStar;
   end
end
dz0 = zeros(4,1);   %Don't change this
u = zeros(3,1);   %Or this.

%%%% Run simulation:
tSpan = [0,5];
x0 = [z0;dz0];
rhs = @(t,x)([x(5:8); dynamics_ds(x(1:4),x(5:8),u,p)]);

%%%% Extract solution:
sol = ode45(rhs,tSpan,x0);

t = linspace(tSpan(1), tSpan(2), 200)';
x = deval(sol,t);
z = x(1:4,:);
dz = x(5:8,:); 

[pHip, p1, p2] = getJointPos_ds(z,p);
[dpHip, dp1, dp2] = getJointVel_ds(z,dz,p);

[ddz, f] = dynamics_ds(z,dz,u,p);

%%%% Plot and Animate
traj.config.dyn = p;
traj.phase = makeStruct(t,z,dz,u,f,pHip,p1,p2,dpHip,dp1,dp2);
plotTraj(traj);

X = [z;pHip;p1;p2];
P.plotFunc = @(t,z,P)drawRanger(z,p); 
P.speed = 0.5;
P.figNum = 162;
animate(t,X,P);

