function model = initializeModel()
%
% This function returns the model (parameters) for ranger's physics and
% controller simulation.

model.control.ank.qSpring = 0.0;
model.control.ank.kSpring = 0.0;
model.control.ank.kMotor = 0.612;
model.control.ank.uMax = 3.6;  %3.6 Nm continuous, 4.8 Nm peak

model.control.hip.qSpring = 0.0;
model.control.hip.kSpring = 8.045;
model.control.hip.kMotor = 1.188;
model.control.hip.uMax = 4.5;   %4.5 Nm continuous, 6.0 Nm peak

model.dyn.g = 9.81; % acceleration due to gravity
model.dyn.l = 0.96; % leg length (hip joint to foot joint)
model.dyn.d = 0.14; % distance between foot joint and virtual center of foot
model.dyn.r = 0.2; % radius of circular arc on foot
model.dyn.c = 0.15; % distance along leg from hip to CoM
model.dyn.m = 4.96; % mass of each leg
model.dyn.I = 0.45; % moment of inertia of the leg about its center of mass
model.dyn.Ifoot = 0.002;  % moment of inertia of the foot about the foot joint
model.dyn.b = 0.0;   %Rolling friction at ground
model.dyn.Phi = 1.85;   %Ankle angle orientation constant

model.dyn.ground = zeros(1,6);  %Flat ground
model.dyn.dt = 0.001;  % Very small for now to deal with foot inertia. 
model.dyn.cstWn = 0.6;   %Normalized by (0.5/dt); Constraint solver stiffness
model.dyn.cstXi = 2.0;   % Damping ratio in constraint solver.
model.dyn.maxStepDuration = 2.0;  % Terminate steps longer than this.
model.dyn.minStepDuration = 0.25;   % Do not allow steps shorter than this.
model.dyn.nDataPerStep = ceil(model.dyn.maxStepDuration/model.dyn.dt);  %For memory allocation

model.dist.t0 = 0.0;
model.dist.t1 = 1.0;
model.dist.fx = 0.0;
model.dist.fy = 0.0;

model.motor.ank.R = 1.3; %ohms (terminal resistance)
model.motor.ank.Vc = 0.7; %volts (contact voltage drop)
model.motor.ank.K = 0.018; %Nm/A (torque constant)
model.motor.ank.G = 34;%{66 -> hip, 34 -> ankle} gearbox ratio
model.motor.ank.c1 = 0.00; %Nms/rad (viscous friction)   Pranav: 0.0 Nms/rad
model.motor.ank.c0 = 0.01; %Nm (constant friction)  Pranav: 0.01 Nm
model.motor.ank.mu = 0.1; %(current-depentent constant friction)   Pranav: 0.1

model.motor.ank.Imax = 6.0; %maximum allowable motor current (3.1 continuous, 8.0 peak)

model.motor.ank.alpha = 0.5;  %Smoothing parameter for sign() function
model.motor.ank.xSpring = 1.662;
model.motor.ank.kSpring = 0.134;

model.motor.hip = model.motor.ank;  %%% <-- KEY LINE

model.motor.hip.G = 66;
model.motor.hip.xSpring = 0.0;
model.motor.hip.kSpring = 8.045;

model.motor.overheadPower = 4.5;  %(Watts) estimated power consumed by computers and sensors
model.motor.Phi = model.dyn.Phi;  %Ankle joint orientation constant

end