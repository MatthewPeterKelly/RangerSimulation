%TEST_getTraj_ds.m
%
% This script tests the trajectory optimization code for double stance, but
% find a trajectory that simply moves the hip forward by some distance
clear; clc;

%%%% Parameters for Dynamics %%%%
config.dyn.g = 9.81; % acceleration due to gravity
config.dyn.l = 0.96; % leg length (hip joint to foot joint)
config.dyn.d = 0.14; % distance between foot joint and virtual center of foot
config.dyn.r = 0.2; % radius of circular arc on foot
config.dyn.c = 0.15; % distance along leg from hip to CoM
config.dyn.w = 0; % distance off leg from main axis to CoM
config.dyn.m = 4.96; % mass of each leg
config.dyn.I = 0.45; % moment of inertia of the leg about its center of mass
config.dyn.Ifoot = 0.05; % moment of inertia of the foot about the foot joint

config.dyn.angleToe = 30*(pi/180);
config.dyn.angleHeel = -5*(pi/180);

%%%% Constraints on the step %%%%
config.step.hipDist = 0.05;  %Desired hip motion 

config.step.bndDuration = [0.1,0.4];

config.step.init.stanceLegAngle = 0.3;  %Initial stance leg angle

config.step.foot.flipMax = 120*(pi/180);
config.step.foot.flipBnd = [-60,80]*(pi/180);  %Parabolic flip constraint
config.step.leg.maxAngle = 40*(pi/180);

%%%% Optimization settings:
config.opt.loadGuess = 'DATA_trajectoryGuess_ds.mat'; % {'', 'DATA_trajectoryGuess.mat'};
config.opt.nSegment = 10;  %Number of segments
config.opt.nColPts = 4;  %Number of collocation points per segment
config.opt.nlp.maxIter = 250;
config.opt.nlp.tol = 1e-6;
config.opt.mesh.maxIter = 1;
config.opt.mesh.tol = 1e-4;

config.opt.nlp.tol = 1e-6;

%%%% THIS IS THE KEY LINE:
traj = getTraj_ds(config);

t = traj.phase.t;
X = [traj.phase.z; traj.phase.pHip; traj.phase.p1; traj.phase.p2];
P.plotFunc = @(t,X,P)drawRanger(X,config.dyn); 
P.speed = 0.2;
P.figNum = 142;
animate(t,X,P);

figure(143);  plotTraj(traj);

%%%% Save the guess for the next trajectory optimization:
save('DATA_trajectoryGuess_ds.mat','traj');

