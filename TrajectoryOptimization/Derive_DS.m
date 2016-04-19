% Derive_Dynamics_DS.m
%
% Derives the equations of motion for Ranger, for the double stance phase
% of motion
%
% Author:  Matthew P. Kelly
% Date:  March 31, 2015
%

%%%% NAMING %%%%
%
% q = absolute angle = orientation of rigid body
% dq = ansolute angle rate
% ddq = absolute angle acceleration
%
% 1 = 2d position vector for a joint
% d1 = 2d velocity vector for a joint
% dd1 = 2d acceleration vector for a joint
%
% G = 2d position vector for a center of mass
% dG = 2d velocity vector for a center of mass
% ddG = 2d acceleration vector for a center of mass
%
% z = vector of absolute angles
% dz = vector of absolute angle rates
% ddz = vector of absolute angle accelerations
%
% e = unit vector pointing down when corresponding angle is zero
% n = unit vector normal to e, following positive sign convention
%

%%%% ANGLES %%%%
% By convention, we will measure the angle of each of the four rigid bodies
% (two feet and two legs) using absolute angle in the plane. For the legs,
% an angle of zero corresponds to the configuration where the ankle joint
% is directly below the hip joint. For the feet, an angle of zero
% corresponds to the configuration where the ankle joint is directly below
% the virtual center of the foot's circular arc.

clc; clear;

disp('Loading symbolic toolbox and initializing variables...')

%%%% States and derivatives %%%%
syms qf1 qf2 'real' % absolute angle of the feet
syms ql1 ql2 'real' % absolute angle of the legs

syms dqf1 dqf2 dql1 dql2 'real' %abs. angular rates
syms ddqf1 ddqf2 ddql1 ddql2 'real' %abs. angular accelerations

%%%% Joint Torques %%%%
syms u1 uHip u2 'real' % torques at feet and hip joints

%%%% Parameters %%%%
syms g 'real' % acceleration due to gravity
syms l 'real' % leg length (hip joint to foot joint)
syms d 'real' % distance between foot joint and virtual center of foot
syms r 'real' % radius of circular arc on foot
syms c 'real' % distance along leg from hip to CoM
syms m 'real' % mass of each leg
syms I 'real' % moment of inertia of the leg about its center of mass
syms Ifoot 'real' % moment of inertia of the foot about the foot joint

%%%% Inertial reference frame %%%%
i = sym([1;0]);
j = sym([0;1]);

%%%% Contact forces %%%%
syms F1x F1y 'real' % Contact forces at foot one
syms F2x F2y 'real' % Contact forces at foot two
F1 = F1x*i + F1y*j;
F2 = F2x*i + F2y*j;

%%%% Contact points (passed from contact solver) %%%%
% qc = the angle of the contact point in the frame of the foot.  For
% example, suppose that the foot is in the normal rolling contact mode, on
% flat ground. Then qc1 = -qf1.
% syms qc1 qc2 'real'
qc1 = -qf1; qc2 = -qf2;   %Assume flat ground

%%%% State Vectors %%%%
z = [qf1; qf2; ql1; ql2];
dz = [dqf1; dqf2; dql1; dql2];
ddz = [ddqf1; ddqf2; ddql1; ddql2];

%%%% Unit vectors %%%%
ef1 = cos(qf1)*(-j) + sin(qf1)*(i); %Virtual center 1 to foot joint 1
ef2 = cos(qf2)*(-j) + sin(qf2)*(i); %Virtual center 2 to foot joint 2

el1 = cos(ql1)*(-j) + sin(ql1)*(i); %Hip to foot joint 1
el2 = cos(ql2)*(-j) + sin(ql2)*(i); %Hip to foot joint 2

disp('Solving kinematics...')

%%%% Position Vectors %%%%
syms x0 'real'  % Dummy variable related to foot spacing

% x==0 is set to be the contact point when qf1 == 0
p1c = (-qf1*r)*i;  %Contact point for foot one 
p2c = (x0-qf2*r)*i; %Contact point for foot two

p1v = p1c + r*j;  %virtual center of foot one - rolling contact
p2v = p2c + r*j;  %Virtual center of foot two - rolling contact

p1 = p1v + d*ef1;  %Joint of foot one joint
p2 = p2v + d*ef2;  %Joint of foot two joint

pHip1 = p1 - l*el1; %Position of hip joint (starting from foot one)
pHip2 = p2 - l*el2; %Position of hip joint (starting from foot two)

G1 = pHip1 + c*el1;  %Center of mass of leg one
G2 = pHip2 + c*el2;  %Center of mass of leg two



%%%% Velocity Vectors %%%%

dp1 = jacobian(p1,z)*dz;
dp2 = jacobian(p2,z)*dz;

dG1 = jacobian(G1,z)*dz;
dG2 = jacobian(G2,z)*dz;

dpHip1 = jacobian(pHip1,z)*dz;
dpHip2 = jacobian(pHip2,z)*dz;


%%%% Acceleration Vectors %%%%

ddp1 = jacobian(dp1,[z;dz])*[dz;ddz];
ddp2 = jacobian(dp2,[z;dz])*[dz;ddz];

ddG1 = jacobian(dG1,[z;dz])*[dz;ddz];
ddG2 = jacobian(dG2,[z;dz])*[dz;ddz];

ddpHip1 = jacobian(dpHip1,[z;dz])*[dz;ddz];
ddpHip2 = jacobian(dpHip2,[z;dz])*[dz;ddz];

ddG = 0.5*(ddG1 + ddG2);

%%%% Define a function for doing '2d' cross product: dot(a x b, k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            DYNAMICS                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Angular momentum balance == AMB

disp('Writing equations of motion...');

%%%% Linear momentum balance for entire system:
sumForce_sys = F1 + F2 - 2*m*g*j;
sumAccel_sys = 2*m*ddG;

lmb_sys_x = dot(i,sumForce_sys - sumAccel_sys);
lmb_sys_y = dot(j,sumForce_sys - sumAccel_sys);

%%%% AMB of leg one about hip joint

sumMom_pHipOne = ...    % ( r X mg ) --> external torques on system (gravity)
    cross2d( G1 - pHip1, -m*g*j ) + ...        % Leg One
    cross2d( p1c - pHip1, F1 ) + ...  %Contact forces on foot one
    -uHip;  %Hip torque (acting on swing leg)

sumHdot_pHipOne = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G1 - pHip1, m*ddG1 ) + I*ddql1 + ...  %Leg One
    Ifoot*ddqf1;  %Foot One

amb_legOne = simplify(sumMom_pHipOne-sumHdot_pHipOne);

%%%% AMB of leg two about hip joint

sumMom_pHipTwo = ...    % ( r X mg ) --> external torques on system (gravity)
    cross2d( G2 - pHip2, -m*g*j ) + ...        % Leg Two
    cross2d( p2c - pHip2, F2 ) + ...  %Contact forces on foot two
    uHip;  %Hip torque (acting on swing leg)

sumHdot_pHipTwo = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G2 - pHip2, m*ddG2 ) + I*ddql2 + ...  %Leg Two
    Ifoot*ddqf2;  %Foot Two

amb_legTwo = simplify(sumMom_pHipTwo-sumHdot_pHipTwo);

%%%% AMB of foot one about ankle joint

sumMom_p1 = ...    % ( r X mg ) --> external torques on system (gravity)
    cross2d( p1c - p1, F1 ) + ...  %Contact forces on foot one
    u1;   %Ankle Torque 1 (acting on foot)

sumHdot_p1 = ... % (r x ma) + (I*alpha) --> inertial terms
    Ifoot*ddqf1;  %Foot Two

amb_footOne = simplify(sumMom_p1-sumHdot_p1);

%%%% AMB of foot two about ankle joint

sumMom_p2 = ...    % ( r X mg ) --> external torques on system (gravity)
    cross2d( p2c - p2, F2 ) + ...  %Contact forces on foot two
    u2;   %Ankle Torque 1 (acting on foot)

sumHdot_p2 = ... % (r x ma) + (I*alpha) --> inertial terms
    Ifoot*ddqf2;  %Foot Two

amb_footTwo = simplify(sumMom_p2-sumHdot_p2);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Constraint Equations                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Hip positions must match:
cst_hip_pos = pHip1 - pHip2;

% Hip velocity must match:
cst_hip_vel = dpHip1 - dpHip2;

% Hip accelerations must match:
cst_hip_acc = ddpHip1 - ddpHip2;

    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Kinematics                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
pHip = 0.5*(pHip1 + pHip2);
matlabFunction(pHip,p1,p2,...
    'file','autoGen_getJointPos.m',...
    'vars',{...
    qf1,qf2,ql1,ql2,...
    r,d,l,x0},...
    'outputs',{'pHip','p1','p2'});

dpHip = 0.5*(dpHip1 + dpHip2);
matlabFunction(dpHip,dp1,dp2,...
    'file','autoGen_getJointVel.m',...
    'vars',{...
    qf1,qf2,ql1,ql2,...
    dqf1,dqf2,dql1,dql2,...
    r,d,l},...
    'outputs',{'dpHip','dp1','dp2'});

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Write Double Stance Dynamics                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

eqns = [
    lmb_sys_x;
    lmb_sys_y;
    amb_legOne;
    amb_legTwo;
    amb_footOne;
    amb_footTwo;
    cst_hip_acc];

vars = [ddz;F1x;F1y;F2x;F2y];

[M, f] = equationsToMatrix(eqns,vars);

% Index tricks to make vectorization work...
Mii = find(M);  %Extract indicies of non-zero elements 
Mnz = M(Mii);

%%%% Write dynamics function
syms one 'real'  % Dummy constraint for vectorization
matlabFunction(Mnz*one,Mii,f*one,...
    'file','autoGen_dynamics_ds.m',...
    'vars',{...
        qf1, qf2, ql1, ql2,...  %Configuration
        dqf1, dqf2, dql1, dql2,... % Rates
        u1, u2, uHip,... % Control
        g,l,d,r,c,m,I,Ifoot,x0,one},... % Parameters
    'outputs',{'Mnz','Mii','f'});


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%             Project state to constraint manifold:                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Fix hip height constraint:
% Linearized constraint model:
% h = h(z0) + dh(z0)*(zStar-z0) --> 0;
h = cst_hip_pos(2);  
dh = jacobian(h,z);
zStar = z-pinv(dh)*h;
matlabFunction(zStar,...
    'file','autoGen_projectConstraints.m',...
    'vars',{...
        qf1, qf2, ql1, ql2,...  %Configuration
        l,d},... % Parameters
    'outputs',{'zStar'});

%%%% Compute the foot reference distance (x0)
soln_x0 = solve(cst_hip_pos(1),x0);
matlabFunction(soln_x0,...
    'file','autoGen_getFootRefDist.m',...
    'vars',{...
        qf1, qf2, ql1, ql2,...  %Configuration
        l,d,r},... % Parameters
    'outputs',{'x0'});



