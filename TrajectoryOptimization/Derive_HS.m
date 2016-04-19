% Derive_HS.m
%
% Derives the heel-strike equations for Ranger
%
% Author:  Matthew P. Kelly
% Date:  March 20, 2015
%

%%%% NAMING %%%%
%
% a = after collision
% b = before collision
%
% q = absolute angle = orientation of rigid body
% dq = ansolute angle rate
%
% 1 = 2d position vector for a joint
% d1 = 2d velocity vector for a joint
% dd1 = 2d acceleration vector for a joint
%
% G = 2d position vector for a center of mass
% dG = 2d velocity vector for a center of mass
%
% z = vector of absolute angles
% dz = vector of absolute angle rates
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

syms dqf1b dqf2b dql1b dql2b 'real' %abs. angular rates (before)
syms dqf1a dqf2a dql1a dql2a 'real' %abs. angular rates (after)

syms x y 'real'; %Cartesian position of the hip joint
syms dxb dyb 'real'; %Cartesian velocity of hip joint (before)
syms dxa dya 'real'; %Cartesian velocity of hip joint (after)

%%%% Parameters %%%%
syms g 'real' % acceleration due to gravity
syms l 'real' % leg length (hip joint to foot joint)
syms d 'real' % distance between foot joint and virtual center of foot
syms r 'real' % radius of circular arc on foot
syms c 'real' % distance along leg from hip to CoM
syms m 'real' % mass of each leg
syms I 'real' % moment of inertia of the leg about its center of mass
syms Ifoot 'real' % moment of inertia of the foot about the foot joint

%%%% State Vectors %%%%
z = [x; y; qf1; qf2; ql1; ql2];
dzb = [dxb; dyb; dqf1b; dqf2b; dql1b; dql2b];  %before collision
dza = [dxa; dya; dqf1a; dqf2a; dql1a; dql2a];  %after collision

%%%% Unit Vectors %%%%
i = sym([1;0]);
j = sym([0;1]);

ef1 = cos(qf1)*(-j) + sin(qf1)*(i); %Virtual center 1 to foot joint 1
ef2 = cos(qf2)*(-j) + sin(qf2)*(i); %Virtual center 2 to foot joint 2

el1 = cos(ql1)*(-j) + sin(ql1)*(i); %Hip to foot joint 1
el2 = cos(ql2)*(-j) + sin(ql2)*(i); %Hip to foot joint 2

%%%% Position Vectors %%%%

pHip = x*i + y*j;

p1 = pHip + l*el1;
p2 = pHip + l*el2;

G1 = pHip + c*el1;  %Center of mass of leg one
G2 = pHip + c*el2;  %Center of mass of leg two

p1v = p1 - d*ef1;
p2v = p2 - d*ef2;

p1c = p1v - r*j;
p2c = p2v - r*j;

%%%% Velocity Vectors  (before) %%%%

dpHipb = simplify(jacobian(pHip,z)*dzb);

dp1b = simplify(jacobian(p1,z)*dzb);
dp2b = simplify(jacobian(p2,z)*dzb);

dG1b = simplify(jacobian(G1,z)*dzb);
dG2b = simplify(jacobian(G2,z)*dzb);

dp1cb = simplify(jacobian(p1c,z)*dzb);
dp2cb = simplify(jacobian(p2c,z)*dzb);

%%%% Velocity Vectors  (after) %%%%

dpHipa = simplify(jacobian(pHip,z)*dza);

dp1a = simplify(jacobian(p1,z)*dza);
dp2a = simplify(jacobian(p2,z)*dza);

dG1a = simplify(jacobian(G1,z)*dza);
dG2a = simplify(jacobian(G2,z)*dza);

dp1va = simplify(jacobian(p1v,z)*dza);
dp2va = simplify(jacobian(p2v,z)*dza);

dp1ca = simplify(jacobian(p1c,z)*dza);
dp2ca = simplify(jacobian(p2c,z)*dza);

%%%% Define a function for doing '2d' cross product: dot(a x b, k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            DYNAMICS                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% stance foot is rolling after the collision:
rollingConstraint = dp1va - [-r*dqf1a; 0];

%%%% Angular momentum balance (AMB) of system about stance foot contact (p1c)

%before
sumH_p1cb = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G1 - p1c, m*dG1b ) + I*dql1b + ...  %Leg One
    cross2d( G2 - p1c, m*dG2b ) + I*dql2b + ...  %Leg Two
    Ifoot*dqf1b + ... %Foot One
    Ifoot*dqf2b;  %Foot Two
%after
sumH_p1ca = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G1 - p1c, m*dG1a ) + I*dql1a + ...  %Leg One
    cross2d( G2 - p1c, m*dG2a ) + I*dql2a + ...  %Leg Two
    Ifoot*dqf1a + ... %Foot One
    Ifoot*dqf2a;  %Foot Two

%%%% AMB of system about ankle joint one (p1)
    
%before
sumH_p1b = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G1 - p1, m*dG1b ) + I*dql1b + ...  %Leg One
    cross2d( G2 - p1, m*dG2b ) + I*dql2b+ ...  %Leg Two
    Ifoot*dqf2b;  %Foot Two

%after
sumH_p1a = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G1 - p1, m*dG1a ) + I*dql1a + ...  %Leg One
    cross2d( G2 - p1, m*dG2a ) + I*dql2a + ...  %Leg Two
    Ifoot*dqf2a;  %Foot Two

%%%% AMB of swing leg about hip joint

%before
sumH_pHipb = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G2 - pHip, m*dG2b ) + I*dql2b + ...  %Leg Two
    Ifoot*dqf2b;  %Foot Two

%after
sumH_pHipa = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G2 - pHip, m*dG2a ) + I*dql2a + ...  %Leg Two
    Ifoot*dqf2a;  %Foot Two

%%%% AMB of swing foot about ankle joint

%before
sumH_p2b = ... % (r x ma) + (I*alpha) --> inertial terms
    Ifoot*dqf2b;  %Foot Two

%after
sumH_p2a = ... % (r x ma) + (I*alpha) --> inertial terms
    Ifoot*dqf2a;  %Foot Two

%%%% Collect equations and write as 'linear' system in ddz:
eqns = [...
    rollingConstraint;
   sumH_p1cb -  sumH_p1ca; 
   sumH_p1b -  sumH_p1a; 
   sumH_pHipb -  sumH_pHipa; 
   sumH_p2b -  sumH_p2a]; 


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Solve equations and write files                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

vars = [dxa; dya; dqf1a; dqf2a; dql1a; dql2a];
[M,f] = equationsToMatrix(eqns,vars);

matlabFunction( M, f, 'file', 'autoGen_heelStrike.m',...
    'vars', {qf1,ql1,ql2,...
    dxb,dyb,dqf1b,dqf2b,dql1b,dql2b,...
    I,Ifoot,c,d,l,m,r});
    





