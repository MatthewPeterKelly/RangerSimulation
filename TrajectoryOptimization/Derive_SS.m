% Derive_SS.m
%
% Derives the equations of motion for Ranger, given that one foot is
% rolling on the ground and one foot is in the air 
%
% Author:  Matthew P. Kelly
% Date:  March 16, 2015
%

%%%% NAMING %%%%
% one - 1 - stance leg or foot (on ground)
% two - 2 - swing leg or foot (in air)
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

%%%% Unknown contact forces %%%%
syms F1x F1y 'real' % Contact forces at stance foot 

%%%% State Vectors %%%%
z = [qf1; qf2; ql1; ql2];
dz = [dqf1; dqf2; dql1; dql2];
ddz = [ddqf1; ddqf2; ddql1; ddql2];

%%%% Unit Vectors %%%%
i = sym([1;0]);
j = sym([0;1]);

ef1 = cos(qf1)*(-j) + sin(qf1)*(i); %Virtual center 1 to foot joint 1
ef2 = cos(qf2)*(-j) + sin(qf2)*(i); %Virtual center 2 to foot joint 2

el1 = cos(ql1)*(-j) + sin(ql1)*(i); %Hip to foot joint 1
el2 = cos(ql2)*(-j) + sin(ql2)*(i); %Hip to foot joint 2

%%%% Position Vectors %%%%
% Origin is defined to be the contact point of the stance foot when qf1==0

p1c = -qf1*r*i;  %Contact point for foot one (assume full circle)
p1v = p1c + r*j;  %virtual center of foot one - rolling contact

p1 = p1v + d*ef1;  %Joint of foot one joint

pHip = p1 - l*el1; %Position of hip joint

G1 = pHip + c*el1;  %Center of mass of leg one
G2 = pHip + c*el2;  %Center of mass of leg two

p2 = pHip + l*el2;  %Position of foot two point

p2v = p2 - d*ef2;   %Position of foot two virtual center
p2c = p2v - r*j;    %Position of foot two contact point

%%%% Velocity Vectors %%%%

dp1c = simplify(jacobian(p1c,z)*dz);
dp1 = simplify(jacobian(p1,z)*dz);
dG1 = simplify(jacobian(G1,z)*dz);

dpHip = simplify(jacobian(pHip,z)*dz);

dp2 = simplify(jacobian(p2,z)*dz);
dG2 = simplify(jacobian(G2,z)*dz);

dp2c = simplify(jacobian(p2c,z)*dz);

%%%% Acceleration Vectors %%%%

ddp1c = simplify(jacobian(dp1c,[z;dz])*[dz;ddz]);
ddp1 = simplify(jacobian(dp1,[z;dz])*[dz;ddz]);
ddG1 = simplify(jacobian(dG1,[z;dz])*[dz;ddz]);

ddpHip = simplify(jacobian(dpHip,[z;dz])*[dz;ddz]);

ddp2 = simplify(jacobian(dp2,[z;dz])*[dz;ddz]);
ddG2 = simplify(jacobian(dG2,[z;dz])*[dz;ddz]);

%%%% Define a function for doing '2d' cross product: dot(a x b, k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            DYNAMICS                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Linear momentum balance for center of mass
sumForce = ...
    F1x*i +  F1y*j - 2*m*g*j;
sumLdot = ...
    m*ddG1 + m*ddG2;
lmb_sys = sumForce - sumLdot;

%%%% Angular momentum balance (AMB) of system about stance foot contact (p1c)

sumMom_p1c = ...    % ( r X mg ) --> external torques on system (gravity)
    cross2d( G1 - p1c, -m*g*j ) + ...   % Leg One
    cross2d( G2 - p1c, -m*g*j );        % Leg Two

sumHdot_p1c = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G1 - p1c, m*ddG1 ) + I*ddql1 + ...  %Leg One
    cross2d( G2 - p1c, m*ddG2 ) + I*ddql2 + ...  %Leg Two
    Ifoot*ddqf1 + ... %Foot One
    Ifoot*ddqf2;  %Foot Two

%%%% AMB of system about ankle joint one (p1)

sumMom_p1 = ...    % ( r X mg ) --> external torques on system (gravity)
    cross2d( G1 - p1, -m*g*j ) + ...   % Leg One
    cross2d( G2 - p1, -m*g*j ) + ...        % Leg Two
    -u1;   %Ankle Torque One  (acting on foot)
    
sumHdot_p1 = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G1 - p1, m*ddG1 ) + I*ddql1 + ...  %Leg One
    cross2d( G2 - p1, m*ddG2 ) + I*ddql2 + ...  %Leg Two
    Ifoot*ddqf2;  %Foot Two


%%%% AMB of swing leg about hip joint

sumMom_pHip = ...    % ( r X mg ) --> external torques on system (gravity)
    cross2d( G2 - pHip, -m*g*j ) + ...        % Leg Two
    uHip;  %Hip torque (acting on swing leg)

sumHdot_pHip = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G2 - pHip, m*ddG2 ) + I*ddql2 + ...  %Leg Two
    Ifoot*ddqf2;  %Foot Two

%%%% AMB of swing foot about ankle joint

sumMom_p2 = ...    % ( r X mg ) --> external torques on system (gravity)
    u2;   %Ankle Torque 1 (acting on foot)

sumHdot_p2 = ... % (r x ma) + (I*alpha) --> inertial terms
    Ifoot*ddqf2;  %Foot Two

%%%% Collect equations and write as 'linear' system in ddz:
eqns = [...
   sumMom_p1c -  sumHdot_p1c; 
   sumMom_p1 -  sumHdot_p1; 
   sumMom_pHip -  sumHdot_pHip; 
   sumMom_p2 -  sumHdot_p2]; 

[MM, f] = equationsToMatrix(eqns,ddz);

% Index tricks to make vectorization work...
Mii = find(MM);  %Extract indicies of non-zero elements 
Mnz = MM(Mii);

%%%% Write dynamics function
matlabFunction(Mnz,Mii,f,...
    'file','autoGen_dynamics_ss.m',...
    'vars',{...
        qf1, ql1, ql2, dqf1, dql1, dql2,... % States  (no dep. on qf2, dqf2)
        u1, u2, uHip,... % Control
        g,l,d,r,c,m,I,Ifoot},... % Parameters
    'outputs',{'Mnz','Mii','f'});

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       solve for contact forces                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

[A,b] = equationsToMatrix(lmb_sys,[F1x;F1y]);
soln_f1 = A\b;

%%%% Write contact force solver
matlabFunction(soln_f1(1), soln_f1(2),...  % Contact force
    'file','autoGen_contactForce_ss.m',...
    'vars',{...
         qf1, ql1, ql2,...
         dqf1, dql1, dql2,...
         ddqf1, ddql1, ddql2,...
        c,g,m,l,d,r},... % Parameters
    'outputs',{'f1x','f1y'});


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Other Useful Stuff                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Write kinematics
matlabFunction(p1,pHip,p2,...  % Positions
    dp1,dpHip,dp2,...  % Velocity
    'file','autoGen_kinematics_ss.m',...
    'vars',{...
         qf1, ql1, ql2, dqf1, dql1, dql2,... % States  (no dep. on qf2, dqf2)
        l,d,r},... % Parameters
    'outputs',{'p1','pHip','p2','dp1','dpHip','dp2'});

%%%% Write kinematics of contact points:
matlabFunction(p2c,dp2c,...
    'file', 'autoGen_contactKinematics.m',...
    'vars',{...
    qf1,qf2,ql1,ql2,...
    dqf1,dqf2,dql1,dql2,...
    l,d,r},... % Parameters
    'outputs',{'p2c','dp2c'});

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%          Convert between control and state variables                    %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
syms p q1 q2 q3 'real'  % Different versions of the coordinate system

coordinateMap = [   p + ql1;
q1 - (qf1-ql1);
q2 - (ql2-ql1);
q3 - (qf2-ql2)];

ctrlVars = solve(coordinateMap,[p;q1;q2;q3]);
stateVars = solve(coordinateMap,[qf1;qf2;ql1;ql2]);

matlabFunction(ctrlVars.p,ctrlVars.q1,ctrlVars.q2,ctrlVars.q3,...
    'file','autoGen_ctrlVars.m',...
    'vars',{qf1,qf2,ql1,ql2},...
    'outputs',{'p','q1','q2','q3'});
matlabFunction(stateVars.qf1,stateVars.qf2,stateVars.ql1,stateVars.ql2,...
    'file','autoGen_stateVars.m',...
    'vars',{p,q1,q2,q3},...
    'outputs', {'qf1','qf2','ql1','ql2'});



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Feedback Linearization                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Write out dynamics in matrix form:   D*ddz + G = B*u
u = [u1; uHip; u2];

% D*ddq = tmp = B*u - G
[D,tmp] = equationsToMatrix(eqns,ddz); %check: simplify(eqns - (D*ddz-tmp))
[B,G] = equationsToMatrix(tmp,u); %check: simplify(eqns - (D*ddz-(B*u-G)))

%%%% Write out function to compute matricies:
matlabFunction(D,G,B,...
    'file','autoGen_dyn_ss_mat.m',...
    'vars',{...
        qf1, ql1, ql2, dqf1, dql1, dql2,... % States  (no dep. on qf2, dqf2)
        g,l,d,r,c,m,I,Ifoot},... % Parameters
    'outputs',{'D','G','B'});
