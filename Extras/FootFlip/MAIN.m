% MAIN.m
%
% This script derives the chebyshev weights for the foot flip trajectory
clc; clear;

% Angles that the trajectory sweeps
p.low = -1;
p.mid = 2;
p.upp = 0;

% Number of points along trajectory
p.n = 17;  % p.grid must be: odd, >= 7

% How to distribute curvature:
p.shape = 1.0;  %0.5 -> more uniform curvature, 1.5 -> agressive 

% Compute the trajectory via optimization
traj = footFlipTraj(p);

% interpolate:
tt = linspace(traj.time(1),traj.time(end),250);
yy = chebyshevInterpolate(traj.angle,tt,traj.domain);

% Plot:
figure(1); clf; hold on;
plot(traj.time, traj.angle,'k.','MarkerSize',15);
plot(tt,yy)
