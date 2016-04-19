%TEST_getTraj.m
%
% This script tests the trajectory optimization code
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
config.dyn.Ifoot = 0.01; % moment of inertia of the foot about the foot joint
config.dyn.wn = sqrt(config.dyn.g/config.dyn.l);

config.dyn.angleToe = 30*(pi/180);
config.dyn.angleHeel = -5*(pi/180);

%%%% Constraints on the step %%%%
config.step.length = 0.4;  %Desired step length
config.step.time = 0.4;  %Desired step time

config.step.foot.toe = -65*(pi/180);
config.step.foot.heel = 10*(pi/180);
config.step.foot.flip = 120*(pi/180);
config.step.leg.maxAngle = 40*(pi/180);

%%%% Optimization settings:
config.opt.loadGuess = '';  %DATA_trajectoryGuess_ss.mat'; % {'', 'DATA_trajectoryGuess.mat'};
config.opt.nSegment = 10;  %Number of segments
config.opt.nColPts = 4;  %Number of collocation points per segment
config.opt.nlp.maxIter = 250;
config.opt.nlp.tol = 1e-6;
config.opt.mesh.maxIter = 0;
config.opt.mesh.tol = 1e-4;

%%%% THIS IS THE KEY LINE: %%%%
traj = getTraj_ss(config);





%%%% NOTES: %%%%
%
% The trajectory tracking controller seems to fail because of the high
% accelerations at the beginning and end of the trajectory. In HZD, they
% get around this by doing some sort of trajectory optimization in phase
% space.
%
% Somehow we need to find a feasible trajectory that is smooth, but this is
% difficult to with ranger because it has extremely small ground clearance
% and ligth feet. The result is that the foot flip trajectories need to be
% quick.
%
% It seems that there might be something funny happening with the
% heel-strike function, so that could be a place to look for errors.
%
%
















%%%% construct a reference trajectory:
traj.ref.nGrid = 10; %number of points to store reference trajectory at
traj.ref.wn = 10*config.dyn.wn; %natural frequency of the controller
traj.ref.xi = 0.95; %damping ratio of the controller
%[1, nConfig] = mapping from configuration to phase
traj.ref.c = [0,0,1,0]; % phase = stance leg angle
%[nMeasure, nConfig] = mapping from configuration to measurement
traj.ref.H = [...
    1,0,-1,0;   %stance foot wrt stance leg
    0,1,0,-1;   %swing foot wrt swing leg
    0,0,-1,1];   %swing leg wrt stance leg
traj = buildRefTraj(traj);

%%%% Animation
t = traj.phase.t;
X = [traj.phase.z; traj.phase.pHip; traj.phase.p1; traj.phase.p2];
P.plotFunc = @(t,X,P)drawRanger(X,config.dyn); 
P.speed = 0.05;
P.figNum = 148;
% animate(t,X,P);

%%%% Plotting
plotTraj(traj);
plotCtrl(traj);

% save('DATA_trajectoryGuess_ss.mat','traj');

