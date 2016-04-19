% Derive_Dynamics.m
%
% Derives the equations of motion for Ranger, for the floating base model
% as well as all contact modes.
%
% Author:  Matthew P. Kelly
% Date:  March 23, 2015
%
% Updated: May 16, 2015
% --> Added analytic derivatives
% --> Updated sign and naming conventions
%
% MATLAB Version: 8.5.0.197613 (R2015a)
%

%%%% NAMING %%%%
% 
% th = absolute angle of leg = orientation of rigid body
% dth = ansolute angle rate of leg
% ddth = absolute angle acceleration of leg
%
% phi = absolute angle of foot = orientation of rigid body
% dphi = ansolute angle rate of foot
% ddphi = absolute angle acceleration of foot
%
% p = 2d position vector for a joint
% dp = 2d velocity vector for a joint
% ddp = 2d acceleration vector for a joint
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
% 0 (zero) = Outer leg
% 1 (Out) = Inner leg
%
%
%%%% ANGLES %%%%
% By convention, we will measure the angle of each of the four rigid bodies
% (Inn feet andinnerlegs) using absolute angle in the plane. For the legs,
% an angle of zero corresponds to the configuration where the ankle joint
% is directly below the hip joint. For the feet, an angle of zero
% corresponds to the configuration where the ankle joint is directly below
% the virtual center of the foot's circular arc.
%

clc; clear;

disp('Loading symbolic toolbox and initializing variables...')

%%%% States and derivatives %%%%
syms phi1 phi0 'real' % absolute angle of the feet
syms th1 th0 'real' % absolute angle of the legs

syms dphi1 dphi0 dth1 dth0 'real' %abs. angular rates
syms ddphi1 ddphi0 ddth1 ddth0 'real' %abs. angular accelerations

syms x y 'real'; %Cartesian position of the hip joint
syms dx dy 'real'; %Cartesian velocity of hip joint
syms ddx ddy 'real'; %Cartesian acceleration of hip joint

%%%% Joint Torques %%%%
syms u1 'real' %torque between the inner foot and inner leg. +leg, -foot
syms u0 'real' %torque between the outer foot and outer leg. +leg, -foot
syms uHip 'real' % torque between theinnerlegs. +inner, -outer

%%%% Disturbance forces %%%%
syms fx 'real' % Disturbance force applied horizontally at hips
syms fy 'real' % Disturbance force applied vertically at hips

%%%% Parameters %%%%
syms g 'real' % acceleration due to gravity
syms l 'real' % leg length (hip joint to foot joint)
syms c 'real' % distance along leg from hip to CoM
syms m 'real' % mass of each leg
syms I 'real' % moment of inertia of the leg about its center of mass
syms Ifoot 'real' % moment of inertia of the foot about the foot joint
syms b 'real'  % viscous rolling friction

%%%% Inertial reference frame %%%%
i = sym([1;0]);
j = sym([0;1]);

%%%% Contact forces %%%%
syms F1x F1y 'real' % Contact forces at foot Out
syms F0x F0y 'real' % Contact forces at foot Inn
F1 = F1x*i + F1y*j;
F0 = F0x*i + F0y*j;

%%%% Contact points (passed from contact solver) %%%%
% qc = the angle of the contact point in the frame of the foot.  For
% example, suppose that the foot is in the normal rolling contact mode, on
% flat ground. Then qc1 = -phi1.
syms qc1 qc0 'real'
syms rc1 rc0 'real'  %distance from ankle joint to contact point

%%%% State Vectors %%%%
z = [x; y; phi0; phi1; th0; th1];
dz = [dx; dy; dphi0; dphi1; dth0; dth1];
ddz = [ddx; ddy; ddphi0; ddphi1; ddth0; ddth1];

%%%% Unit vectors %%%%

ec0 = cos(phi0 + qc0)*(-j) + sin(phi0 + qc0)*(i); 
ec1 = cos(phi1 + qc1)*(-j) + sin(phi1 + qc1)*(i); %ankle joint to contact point

el0 = cos(th0)*(-j) + sin(th0)*(i); %Hip to foot joint 0
el1 = cos(th1)*(-j) + sin(th1)*(i); %Hip to foot joint 1

%%%% Position Vectors %%%%

pHip = x*i + y*j;

p0 = pHip + l*el0;
p1 = pHip + l*el1;  %Position of the ankle joint

p0c = p0 + rc0*ec0;
p1c = p1 + rc1*ec1;   %position of the contact point

G0 = pHip + c*el0;  %Center of mass of outer leg
G1 = pHip + c*el1;  %Center of mass of inner leg

G = simplify((m*G0+m*G1)/(m+m));  %Center of mass of system (feet are massless)

%%%% Velocity Vectors %%%%

dpHip = simplify(jacobian(pHip,z)*dz);

dp0 = simplify(jacobian(p0,z)*dz);
dp1 = simplify(jacobian(p1,z)*dz);

dG0 = simplify(jacobian(G0,z)*dz);
dG1 = simplify(jacobian(G1,z)*dz);

dG = simplify(jacobian(G,z)*dz);

dp0c = simplify(jacobian(p0c,z)*dz);
dp1c = simplify(jacobian(p1c,z)*dz);

%%%% Acceleration Vectors %%%%

ddpHip = simplify(jacobian(dpHip,[z;dz])*[dz;ddz]);

ddp0 = simplify(jacobian(dp0,[z;dz])*[dz;ddz]);
ddp1 = simplify(jacobian(dp1,[z;dz])*[dz;ddz]);

ddG0 = simplify(jacobian(dG0,[z;dz])*[dz;ddz]);
ddG1 = simplify(jacobian(dG1,[z;dz])*[dz;ddz]);

ddG = simplify(jacobian(dG,[z;dz])*[dz;ddz]);

ddp0c = simplify(jacobian(dp0c,[z;dz])*[dz;ddz]);
ddp1c = simplify(jacobian(dp1c,[z;dz])*[dz;ddz]);

%%%% Define a function for doing '2d' cross product: dot(cross(a,b), k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            DYNAMICS                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Angular momentum balance == AMB

disp('Writing equations of motion...');


%%%% Linear momentum balance for entire system:
sumForce_sys = F0 + F1 - 2*m*g*j + fx*i + fy*j;
sumAccel_sys = 2*m*ddG;

lmb_sys_x = dot(i,sumForce_sys - sumAccel_sys);
lmb_sys_y = dot(j,sumForce_sys - sumAccel_sys);


%%%% AMB of outer leg about hip joint

sumMom_pHipOut = ...    % ( r X mg ) --> external torques on system (gravity)
    cross2d( G0 - pHip, -m*g*j ) + ...        % Leg Out
    cross2d( p0c - pHip, F0 ) + ...  %Contact forces on foot Out
    -b*dphi0 + ...  %viscous rolling friction
    -uHip;  %Hip torque (acting on leginnerleg)

sumHdot_pHipOut = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G0 - pHip, m*ddG0 ) + I*ddth0 + ...  %Leg Out
    Ifoot*ddphi0;  %Foot Out

amb_legOut = simplify(sumMom_pHipOut-sumHdot_pHipOut);

%%%% AMB of inner leg about hip joint

sumMom_pHipInn = ...    % ( r X mg ) --> external torques on system (gravity)
    cross2d( G1 - pHip, -m*g*j ) + ...        % Leg Inn
    cross2d( p1c - pHip, F1 ) + ...  %Contact forces on foot Inn
    -b*dphi1 + ...  %viscous rolling friction
    uHip;  %Hip torque (acting on leg Inn)

sumHdot_pHipInn = ... % (r x ma) + (I*alpha) --> inertial terms
    cross2d( G1 - pHip, m*ddG1 ) + I*ddth1 + ...  %Leg Inn
    Ifoot*ddphi1;  %Foot Inn

amb_legInn = simplify(sumMom_pHipInn-sumHdot_pHipInn);

%%%% AMB of outer foot about ankle joint

sumMom_p0 = ...    % ( r X mg ) --> external torques on system (gravity)
    cross2d( p0c - p0, F0 ) + ...  %Contact forces on foot Out
    -b*dphi0 + ...  %viscous rolling friction
    -u0;   %inner Ankle Torque (acting on foot Out)

sumHdot_p0 = ... % (r x ma) + (I*alpha) --> inertial terms
    Ifoot*ddphi0;  %Foot Inn

amb_footOut = simplify(sumMom_p0-sumHdot_p0);

%%%% AMB of inner foot about ankle joint

sumMom_p1 = ...    % ( r X mg ) --> external torques on system (gravity)
    cross2d( p1c - p1, F1 ) + ...  %Contact forces on foot Inn
    -b*dphi1 + ...  %viscous rolling friction
    -u1;   %Ankle Torque 1 (acting on foot Inn)

sumHdot_p1 = ... % (r x ma) + (I*alpha) --> inertial terms
    Ifoot*ddphi1;  %Foot Inn

amb_footInn = simplify(sumMom_p1-sumHdot_p1);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Kinematics                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

disp('Writing kinematics functions...')

%%%% Write kinematics
matlabFunction(p0,p1,...  % Positions
    dp0,dp1,...  % Velocity
    ddp0,ddp1,...  % Acceleration
    'file','autoGen_kinematics.m',...
    'vars',{...
    x, y, th0, th1,...   %Position, Angles
    dx, dy,  dth0, dth1,...   %Velocity, Rates
    ddx, ddy, ddth0, ddth1,...   %Velocity, Rates
    l},... % Parameters
    'outputs',{...
    'p0',  'p1', ... %position
    'dp0', 'dp1', ...  %velocity
    'ddp0', 'ddp1'}); %acceleration

%%%% Write kinematics of contact point of foot
matlabFunction(p0c,p1c,...  % Positions
    dp0c,dp1c,...  % Velocity
    'file','autoGen_kinematicsContact.m',...
    'vars',{...
    x, y, phi0, phi1, th0, th1,...   %Position, Angles
    dx, dy,  dphi0, dphi1, dth0, dth1,...   %Velocity, Rates
    qc0,qc1, rc0,rc1, ... %relative angle of contact points
    l},... % Parameters
    'outputs',{...
    'p0c',  'p1c', ... %position
    'dp0c', 'dp1c'}); %velocity

%%%% Write acceleration of contact point
matlabFunction(ddp0c,ddp1c,...  % accelerations
    'file','autoGen_kinematicsContactAccel.m',...
    'vars',{...
    phi0, phi1, th0, th1,...   %Position, Angles
    dphi0, dphi1, dth0, dth1,...   %Velocity, Rates
    ddx, ddy,  ddphi0, ddphi1, ddth0, ddth1,...   %acceleration
    qc0,qc1, rc0,rc1, ... %relative angle of contact points
    l},... % Parameters
    'outputs',{...
    'ddp0c',  'ddp1c'}); %% acceleration

%%%% Write function for computing the center of mass:
matlabFunction(G,dG,...
    'file','autoGen_centerOfMass.m',...
    'vars',{...
         x, y, th0, th1,...   %Position, Angles
    dx, dy,  dth0, dth1,...   %Velocity, Rates
    c},...  %parameters
    'outputs',{'G','dG'});

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Contact Solver:                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Acceleration of the virtual center of the foot:
syms ddp1c_x ddp1c_y ddp0c_x ddp0c_y 'real'

disp('Writing single stance outer dynamics...')
%%%% Case Out: (Single Stance Out)
% Assume:
%   ddp0c_x
%   ddp0c_y
%   F1x = 0
%   F1y = 0
% Check:
%   F0x
%   F0y
%   contact_Inn
eqns_s0 = [...
    lmb_sys_x;
    lmb_sys_y;
    amb_legOut;
    subs(amb_legInn,b,sym(0));  % no rolling friction in the air
    amb_footOut;
    subs(amb_footInn,b,sym(0));  % no rolling friction in the air
    ddp0c_x - ddp0c(1);
    ddp0c_y - ddp0c(2)];
vars_s0 = [ddz; F0x; F0y];

% Assume that there are no forces on the inner foot contact point
eqns_s0 = simplify(subs(eqns_s0,{'F1x','F1y'},{sym(0),sym(0)}));
[M_s0, f_s0] = equationsToMatrix(eqns_s0, vars_s0);

M_s0 = simplify(M_s0);
f_s0 = simplify(f_s0);

%%%% Write dynamics function
matlabFunction(M_s0, f_s0,...
    'file','autoGen_dynamics_s0.m',...
    'vars',{...
    th0,th1,phi0, ...   %Position, Angles
    dth0,dth1,dphi0, ...   %Velocity, Rates
    u0, u1, uHip,... % Control
    fx, fy, ... %Disturbance force at hip
    qc0,rc0,... %relative angle of contact points
    ddp0c_x, ddp0c_y,... %Virtual center acceleration
    g,l,c,m,I,Ifoot,b},... % Parameters
    'outputs',{'M_s0','f_s0'});


disp('Writing single stance inner dynamics...')
%%%% Case inner: (Single Stance inner)
% Assume:
%   F0x = 0
%   F0y = 0
%   ddp1c_x
%   ddp1c_y
% Check:
%   contact_Out
%   F1x
%   F1y
eqns_s1 = [...
    lmb_sys_x;
    lmb_sys_y;
    subs(amb_legOut,b,sym(0));  % no rolling friction in the air
    amb_legInn;
    subs(amb_footOut,b,sym(0));  % no rolling friction in the air
    amb_footInn;...
    ddp1c_x - ddp1c(1);
    ddp1c_y - ddp1c(2)];
vars_s1 = [ddz; F1x; F1y];

% Assume that there are no forces on the outer foot contact point
eqns_s1 = simplify(subs(eqns_s1,{'F0x','F0y'},{sym(0),sym(0)}));
[M_s1, f_s1] = equationsToMatrix(eqns_s1, vars_s1);
M_s1 = simplify(M_s1);
f_s1 = simplify(f_s1);

%%%% Write dynamics function
matlabFunction(M_s1, f_s1,...
    'file','autoGen_dynamics_s1.m',...
    'vars',{...
    th0,th1,phi1, ...   %Position, Angles
    dth0,dth1,dphi1, ...   %Velocity, Rates
    u0,u1, uHip,... % Control
    fx, fy, ... %Disturbance force at hip
    qc1,rc1... %relative angle of contact points
    ddp1c_x, ddp1c_y,... %acceleration of virtual center
    g,l,c,m,I,Ifoot,b},... % Parameters
    'outputs',{'M_s1','f_s1'});


disp('Writing flight dynamics...')
%%%% Case Three: (Flight)
% Assume:
%   F1x = 0
%   F1y = 0
%   F0x = 0
%   F0y = 0
% Check:
%   both contact points
%

eqns_fl = [...
    lmb_sys_x;
    lmb_sys_y;
    amb_legOut;
    amb_legInn;
    amb_footOut;
    amb_footInn];

eqns_fl = simplify(subs(eqns_fl,{'F1x','F1y','F0x','F0y','b'},{sym(0),sym(0),sym(0),sym(0),sym(0)}));
[M_fl, f_fl] = equationsToMatrix(eqns_fl,ddz);

M_fl = simplify(M_fl);
f_fl = simplify(f_fl);

%%%% Write dynamics function
matlabFunction(M_fl,f_fl,...
    'file','autoGen_dynamics_fl.m',...
    'vars',{...
    th0, th1,...   %Position, Angles
    dth0,dth1, ...   %Velocity, Rates
    u0,u1,  uHip,... % Control
    fx, fy, ... %Disturbance force at hip
    g,c,m,I,Ifoot},... % Parameters
    'outputs',{'M_fl','f_fl'});



disp('Writing double stance dynamics...')
%%%% Case Four: (Double Stance)
% Assume:
%   ddp1c_x
%   ddp1c_y
%   ddp0c_x
%   ddp0c_y
% Check:
%   F1x
%   F1y
%   F0x
%   F0y
eqns_ds = [...
        lmb_sys_x;
    lmb_sys_y;
    amb_legOut;
    amb_legInn;
    amb_footOut;
    amb_footInn;...
    ddp1c_x - ddp1c(1);
    ddp1c_y - ddp1c(2);
    ddp0c_x - ddp0c(1);
    ddp0c_y - ddp0c(2)];
vars_ds = [ddz; F1x; F1y; F0x; F0y];

[M_ds, f_ds] = equationsToMatrix(eqns_ds, vars_ds);
M_ds = simplify(M_ds);
f_ds = simplify(f_ds);

%%%% Write dynamics function
matlabFunction(M_ds,f_ds,...
    'file','autoGen_dynamics_ds.m',...
    'vars',{...
    phi0,phi1, th0,  th1,...   %Position, Angles
    dphi0,dphi1,dth0,  dth1, ...   %Velocity, Rates
     u0, u1,uHip,... % Control
    fx, fy, ... %Disturbance force at hip
    qc0,qc1, rc1,rc0, ... %relative angle of contact points
    ddp1c_x, ddp1c_y, ddp0c_x, ddp0c_y,... % acceleration of virtual center
    g,l,c,m,I,Ifoot,b},... % Parameters
    'outputs',{'M_ds','f_ds'});



