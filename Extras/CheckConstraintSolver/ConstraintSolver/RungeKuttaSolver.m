%
% This script is a toy problem for understanding how to use a 4th-order
% Runge-Kutta method to resolve a constraint between two rigid objects,
% where the solver can prescribe the acceleration of the contact point.

% p = position (state)
% v = velocity (state)
% a = acceleration (input)
% h = time step


clear; clc;

h = 0.01;  %Time-step

x0 = 0.01;  % Initial position error
v0 = -1.5;  % initial velocity error

xi = 1.0;
wn = 1/h;

nStep = 15;

t = linspace(0,nStep*h,nStep+1);
z = zeros(2,nStep+1);
z(:,1) = [x0;v0];

for i=1:nStep
  
    z(:,i+1) = rungeKuttaStep(h,z(:,i),xi,wn);
    
end

i = 1:nStep;
x = z(1,:);
vv = z(2,:);

figure(12); clf;
subplot(2,1,1);
plot(t,x,'o',t([1,end]),[0,0])
ylabel('position')

subplot(2,1,2);
plot(t,vv,'o',t([1,end]),[0,0])
ylabel('velocity');
