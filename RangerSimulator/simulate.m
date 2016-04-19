function [stateNext,f,c,k,torque, power, current] = simulate(state, ir, cp, cd, dyn, motor, dist)
% [stateNext,f,c,k,torque, power, current] = simulate(state, ir, cp, cd, dyn, motor, dist)
%
% This function computes a single time-step for the Cornell Ranger,
% assuming that it is walking on flat ground, the feet roll without
% slipping, and the coefficient of restitution for the ground is zero. The
% time-step is computed using symplectic euler integration. The contact
% constraints are approximately satisfied on each time step, with errors
% decaying asymtotically over a few time steps.
%
% INPUTS:
% state = [z;dz] state vector
%   z = [6, 1] = full configuration vector
%   dz = [6, 1] = full configuration rate vector
%       (1) = hip horizontal position
%       (2) = hip vertical position
%       (3) = outer foot absolute angle
%       (4) = inner foot absolute angle
%       (5) = outer leg absolute angle
%       (6) = inner leg absolute angle
%
% ir = reference current = [3, 1] = motor current set-point
% cp = proportional gain (current) = [3, 1] = motor controller
% cd = derivative gain (current) = [3, 1] = motor controller
% 	(1) = outer ankle joint torque (+leg, -foot)
% 	(2) = inner ankle joint torque (+leg, -foot)
% 	(3) = hip torque (+inner, -outer)
%
% dyn = parameter struct:
%    .g = acceleration due to gravity
%    .l = leg length (hip joint to foot joint)
%    .d = distance between foot joint and virtual center of foot
%    .r = radius of circular arc on foot
%    .c = distance along leg from hip to CoM
%    .w = distance off leg from main axis to CoM
%    .m = mass of each leg
%    .I = moment of inertia of the leg about its center of mass
%    .Ifoot = moment of inertia of the foot about the foot joint
%    .dt = time step for integration method
%
% motor = struct with motor paramters:
%
%    .ank.R = 1.3; %ohms (terminal resistance)
%    .ank.Vc = 0.7; %volts (contact voltage drop)
%    .ank.K = 0.018; %Nm/A (torque constant)
%    .ank.G = 34;%{66 -> hip, 34 -> ankle} gearbox ratio
%    .ank.c1 = 0; %Nms/rad (viscous friction)
%    .ank.c0 = 0.01; %Nm (constant friction)
%    .ank.mu = 0.1; %(current-depentent constant friction)
%    .ank.Imax = 8.0; %maximum allowable motor current (3.1 continuous, 8.0 peak)
%    .ank.alpha = 0.02;  %Smoothing parameter for sign() function
%    .ank.xSpring = 1.662;
%    .ank.kSpring = 0.134;
%
%    .hip = *.ank  %Most fields are the same, but a few (below) are not:
%    .hip.G = 66;
%    .hip.xSpring = 0.0;
%    .hip.kSpring = 8.045;
%
% dist = [2,1] = column vector of disturbances applied at hip
%   (1) = horizontal component
%   (2) = vertical comonent
%
% OUTPUTS:
%   state = [12 x 1] state vector at next time step
% f = [4, 1] = contact force vector
% f = [f0; f1] = [outer; inner] contact forces
% 	(1) = horizontal component
% 	(2) = vertical component
% c = [4 x 1] = [c0;c1] = [outer;inner] = [x;y;x;y] contact points
% k = [2 x 1] boolean vector [outerFootInContact; innerFootInContact];
% torque = [u0; u1; uHip];   %Realized torques at each joint
% power = power used by each joint, same order as torques
% current = current sent to each joint, same order as torques
%
% NOTES:
%
%   Angle Conventions:
%       - All angles are in absolute reference frame
%       - zero corresponds to:
%           (legs) = ankle joint directly below hip joint
%           (feet) = ankle joint directly below virtual foot center
%

z = state(1:6);
dz = state(7:12);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Kinematics                                    %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Determine where the contact points on each foot would be, if they were in
% contact, and also determine the acceleration of the virtual center of
% each foot that would be consistent with that contact mode.

% Figure out the contact angle for each foot:
[p0,p1] = kinematics(z,[],[],dyn);
phi0 = z(3);  phi1 = z(4);
[qc0, rc0] = getContactInfo(p0,phi0,dyn);
[qc1, rc1] = getContactInfo(p1,phi1,dyn);
qc = [qc0;qc1]; rc = [rc0;rc1];

% Determine the kinematics of each contact point
[c0,c1,dc0,dc1] = kinematicsContact(z,dz,qc,rc,dyn);
c = [c0;c1];  %Contact points for each foot at the start of the time step

% Compute the ground height and normal vector at each contact:
[g0, n0] = groundModel(c0(1),dyn.ground);
[g1, n1] = groundModel(c1(1),dyn.ground);

%Project contact points to surface
c0Next = [c0(1); g0];
c1Next = [c1(1); g1];

% Constraint solver:
h = dyn.dt;
wn = dyn.cstWn/(2*h);   % Stiffness of contact solver (feet and ground interaction)
xi = dyn.cstXi;    % Damping ratio for the contact solver
kp = wn*wn;
kd = 2*xi*wn;
e0 = c0Next - c0;
de0 = -dc0;
e1 = c1Next - c1;
de1 = -dc1;
ddc0 = kp*e0 + kd*de0; 
ddc1 = kp*e1 + kd*de1; 

% Figure out which constraints are in violation:
contactOut = c0(2) < g0;   %Contact on outer foot is active
contactInn = c1(2) < g1;   %Contact on inner foot is active

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Compute dynamics for each mode                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


%Make a guess at which order to attempt contacts:
if contactOut && ~contactInn  %Try single stance outer first
    order = [0,2,1,3];
elseif ~contactOut && contactInn  %Try single stance inner first
    order = [1,2,0,3];
elseif contactOut && contactInn  %Try double stance first
    order = [2,0,1,3];
else
    order = [3,0,2,1];   %Try flight first
end

XX_vln = 0;  % Contact force violation;
XX_stateNext = zeros(12,1);
XX_f = zeros(4,1);
XX_k = false(2,1);
XX_torque = zeros(3,1);
XX_power = zeros(3,1);
XX_current = zeros(3,1);
for i=order
    switch i
        case 0 %single stance outer
            [S0_stateNext, S0_f, S0_valid, S0_vln, S0_torque, S0_power, S0_current]...
                = simulate_s0(z,dz,qc,rc,ddc0,n1,ir, cp, cd, dyn, motor, dist);
            if S0_valid
                stateNext = S0_stateNext;  f = S0_f;  k = [true; false];
                torque = S0_torque; power = S0_power; current = S0_current;
                return;
            elseif S0_vln > 0 && S0_vln < XX_vln
                XX_stateNext = S0_stateNext; XX_f = S0_f;
                XX_k = [true; false];
                XX_torque = S0_torque; XX_power = S0_power; XX_current = S0_current;
            end
        case 1 %single stance - inner legs
            [S1_stateNext, S1_f, S1_valid, S1_vln, S1_torque, S1_power, S1_current]...
                = simulate_s1(z,dz,qc,rc,ddc1,n0,ir, cp, cd, dyn, motor, dist);
            if S1_valid
                stateNext = S1_stateNext;  f = S1_f;  k = [false; true];
                torque = S1_torque; power = S1_power; current = S1_current;
                return;
            elseif S1_vln > 0 && S1_vln < XX_vln
                XX_stateNext = S1_stateNext; XX_f = S1_f;
                XX_k = [false; true];
                XX_torque = S1_torque; XX_power = S1_power; XX_current = S1_current;
            end
            
        case 2 %double stance
            ddc = [ddc0;ddc1];
            n = [n0;n1];
            [DS_stateNext ,DS_f, DS_valid, DS_vln, DS_torque, DS_power, DS_current]...
                = simulate_ds(z,dz,qc,rc,ddc,n,ir, cp, cd, dyn, motor, dist);
            if DS_valid
                stateNext = DS_stateNext;  f = DS_f; k = [true; true];
                torque = DS_torque; power = DS_power; current = DS_current;
                return;
            elseif DS_vln > 0 && DS_vln < XX_vln
                XX_stateNext = DS_stateNext; XX_f = DS_f;
                XX_k = [true; true];
                XX_torque = DS_torque; XX_power = DS_power; XX_current = DS_current;
            end
            
        otherwise %flight
            [FL_stateNext, FL_f, FL_valid, FL_torque, FL_power, FL_current] = ...
                simulate_fl(z,dz,qc,rc,ir, cp, cd, dyn, motor, dist);
            if FL_valid
                stateNext = FL_stateNext;  f = FL_f; k = [false; false];
                torque = FL_torque; power = FL_power; current = FL_current;
                return;
            elseif XX_vln == 0
                XX_stateNext = FL_stateNext; XX_f = FL_f;
                XX_k = [true; true];
                XX_torque = FL_torque; XX_power = FL_power; XX_current = FL_current;
            end
    end
end


% If the code reaches here, then no valid solution was found. This means
% that one of the feet must be slipping. Since we assume no-slip, we get an
% inconsistent solution. To resolve this, we relax the ground contact force
% constraint that demands the contact forces be positive (allow negative
% reaction forces until slipping stops).

stateNext = XX_stateNext;
f = XX_f;
k = XX_k;
torque = XX_torque;
power = XX_power;
current = XX_current;

end


function [zNext, dzNext] = symplecticEuler(z,dz,ddz,h)
%
% This function computes the next position and velocity, using symplectic
% euler integration method.
%

dzNext = dz + h*ddz;
zNext = z + h*dzNext;

end


function [stateNext,f,valid,u, power, current] = simulate_fl(z,dz,qc,rc,ir, cp, cd, dyn, motor, dist)

% Run motor model:
[u, power, current] = motorModel([z;dz], ir, cp, cd, motor);

% Solve the dynamics, assuming flight:
ddz = dynamics_fl(z,dz,u,dist,dyn);

% March forward in time using symplectic euler integration
[z, dz] = symplecticEuler(z,dz,ddz,dyn.dt);

% Update state for next iteration:
stateNext = [z;dz];

[c0,c1] = kinematicsContact(z,dz,qc,rc,dyn);

% Contact forces:
f = zeros(4,1);

% Compute the ground height at the next contact
g0 = groundModel(c0(1),dyn.ground);
g1 = groundModel(c1(1),dyn.ground);

% Check if flight phase is valid:
valid  = c0(2) > g0 && c1(2) > g1;

end



function [stateNext,f,valid,violation, u, power, current] = simulate_ds(z,dz,qc,rc,ddc,n,ir, cp, cd, dyn, motor, dist)

% Run motor model:
[u, power, current] = motorModel([z;dz], ir, cp, cd, motor);

% dynamics
[ddz, f0, f1] = dynamics_ds(z,dz,u,dist,qc,rc,ddc,dyn);
% time-step
[z, dz] = symplecticEuler(z,dz,ddz,dyn.dt);

% Compute the ground height at the next contact
n0 = n(1:2); n1 = n(3:4);

% check assumptions:
fn0 = dot(n0,f0);  %Normal force on foot one
fn1 = dot(n1,f1);  %Normal force on foot two
valid = fn0 > 0 && fn1 > 0;

%Collect forces:
f = [f0;f1];
stateNext = [z;dz];

violation = max(-fn0,-fn1);

end


function [stateNext,f,valid,violation,u,power,current] = simulate_s1(z,dz,qc,rc,ddc1,n1,ir, cp, cd, dyn, motor, dist)

% Run motor model:
[u, power, current] = motorModel([z;dz], ir, cp, cd, motor);

% dynamics
[ddz, f1] = dynamics_s1(z,dz,u,dist,qc,rc,ddc1,dyn);
% time-step
[z, dz] = symplecticEuler(z,dz,ddz,dyn.dt);

c0 = kinematicsContact(z,dz,qc,rc,dyn);

% Check assumptions:
f1n = dot(f1,n1);
valid = c0(2) > groundModel(c0(1),dyn.ground) && f1n > 0;
% pack up forces
f = [0;0;f1];

stateNext = [z;dz];

violation = -f1n;

end


function [stateNext,f,valid,violation,u,power,current] = simulate_s0(z,dz,qc,rc,ddc0,n0,ir, cp, cd, dyn, motor, dist)

% Run motor model:
[u, power, current] = motorModel([z;dz], ir, cp, cd, motor);

% dynamics
[ddz, f0] = dynamics_s0(z,dz,u,dist,qc,rc,ddc0,dyn);
% time-step
[z, dz] = symplecticEuler(z,dz,ddz,dyn.dt);

[~,c1] = kinematicsContact(z,dz,qc,rc,dyn);

stateNext = [z;dz];
f = [f0; 0;0];

% check assumption:
f0n = dot(f0,n0);
valid = c1(2) > groundModel(c1(1),dyn.ground) && f0n > 0;

% Check relaxed assumptiona:
violation = -f0n;

end


