% Derive the equations of motion for a series elastic actuator that is
% positioned between a rolling disk and an inverted pendulum. This set-up
% is meant to mimic a single foot and leg of Cornell Ranger.
%
%

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Initialize Variables and Parameters                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% NAMING:
%   _f = foot (rolling disk)
%   _m = motor
%   _l = leg (inverted pendulum)

% Physical parameters:
syms Rad 'real'  %Radius of the disk
syms Len 'real'  % distance from joint to the leg CoM
syms Ecc 'real'  % Distance between virtual disk center and ankle joint
syms g 'real'  %gravity acceleration
syms Jf 'real'  %Moment of inertia of the foot (disk) (mass = zero)
syms Jm 'real'  %Motor Rotor inertia
syms Jl 'real'  %Moment of inertia of the leg about its CoM
syms m 'real'  %Mass of the leg
syms Km 'real'  % Motor torque constant
syms Ks 'real'  % Spring constant coupling foot to motor
syms Ga 'real'  % transmission ratio between rotor and output shaft on motor
syms C0 'real'   % motor friction paramter ( constant )
syms C1 'real'   % Motor friction parametr ( damping )
syms Mu 'real'  % motor friction torque-dep
syms Smooth 'real'  % Friction smoothing term
syms Cf 'real'   % Viscous rolling friction

% System inputs:
syms Ia 'real'  % Current into the motor

% States:   (absolute angles)
syms qf dqf ddqf 'real'  %Foot (disk) angle
syms qm dqm ddqm 'real'  %Motor angle
syms ql dql ddql 'real'  %Leg angle


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Set up useful expressions and functions                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Define State vectors:
z = [qf; qm; ql];
dz = [dqf; dqm; dql];
ddz = [ddqf; ddqm; ddql];

% A useful function for taking derivatives, making use of the chain rule:
derivative = @(x)( jacobian(x,[z;dz])*[dz;ddz] );

% Define a function for doing '2d' cross product: dot(a x b, k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

% Exponential smoothing of abs() and signum()
Sgn = @(x)( tanh(x/Smooth) );  %Smooth signum() function
Abs = @(x)( x*tanh(x/Smooth) );  %Smooth abs() function

% Friction term in motors:
Tf = @(I,dq,G)( C1*dq + C0*Sgn(dq) + Mu*G*Km*Abs(I)*Sgn(dq) );

% Torque due to spring between motor and foot (disk)
Ts = -Ks*(qf-qm);

% Torque acting on motor from leg (pendulum)
Tm = Ga*Km*Ia - Tf(Ia, dqm-dql, Ga);

% Torque opposing foot rolling:
Td = -Cf*dqf;



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Unit Vectors                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Inertial reference frame:
i = sym([1;0]);
j = sym([0;1]);

% Unit vector from disk center to the joint:
ef = cos(qf)*(-j) + sin(qf)*(i);

% Unit vector from pendulum CoM to the joint:
el = cos(ql)*(-j) + sin(ql)*(i);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Kinematics                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

pv = -Rad*qf*i + Rad*j;   %Virtual center of the disk

pc = pv - Rad*j;  % Contact point

p0 = pv + ef*Ecc;   %Joint

pg = p0 - el*Len;   %Leg CoM
dpg = derivative(pg);
ddpg = derivative(dpg);



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Dynamics                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% AMB System @ pc
inertial_A = Jf*ddqf + Ga*Ga*Jm*ddqm + Jl*ddql + cross2d(pg-pc,m*ddpg);
torques_A = Td + cross2d(pg-pc,-m*g*j);
eqnA = inertial_A-torques_A;

%%%% AMB motor @ p0
inertial_B = Ga*Ga*Jm*ddqm;
torques_B = Tm - Ts;
eqnB = inertial_B - torques_B;

%%%% AMB Leg @ p0
inertial_C = Jl*ddql + cross2d(pg-p0,m*ddpg);
torques_C = -Tm + cross2d(pg-p0,-m*g*j);
eqnC = inertial_C - torques_C;

%%%% Collect eqns:
eqns = [eqnA;eqnB;eqnC];
[AA,bb] = equationsToMatrix(eqns,ddz);
soln = simplify(AA\bb);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Other Useful Stuff                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Meff = jacobian(soln,Ia);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Write function files                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


matlabFunction(soln,...
    'file','autoGen_dynamics.m',...
    'vars',{qf,qm,ql,dqf,dqm,dql,Ia,...
    Rad,Len,Ecc,g,Jf,Jm,Jl,m,Km,Ks,Ga,C0,C1,Mu,Smooth,Cf},...
    'outputs',{'ddz'});





