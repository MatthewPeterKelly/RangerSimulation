% SymplecticController.m
%
% This script is for experimenting with contact and collision resolution in
% a simulator implemented using symplectic euler integration. Given that a
% collision is imminent, it is not clear what acceleration (impulse) to
% request. You can either fix the position or velocity, but not both.
% Either case seems to cause funny oscillations. Perhaps a damped approach
% is better.

% p = position (state)
% v = velocity (state)
% a = acceleration (input)
% h = time step

% vNext = v + h*a;
% xNext = x + h*vNext;

% xNext = x + h*(v + h*a)
% vNext = v + h*a

% z = [x;v];  u = a;
% zNext = [1, h; 0, 1]*z + [h;h*h]*u

clear; clc;

h = 0.01;
x0 = 0.02;
v0 = -2.5;

A = [1, h;  %x
    0, 1];  %v
B = [h*h;  %u
    h];

nStep = 10;

zz = zeros(2,nStep);
uu = zeros(1,nStep);
zz(:,1) = [x0;v0];

for i=2:nStep
    z = zz(:,i-1);
    x = z(1); v = z(2);
    
%     u = -(x + h*v)/(h*h);  %Deadbeat
    
     u = -(sqrt(0.5)*x + h*v)/(h*h);  %Exponential
    
    uu(i-1) = u;
    zz(:,i) = A*z+B*u;
end

fprintf('Max Acceleration: %5.5f\n',max(abs(uu)));

i = 1:nStep;
xx = zz(1,:);
vv = zz(2,:);

figure(12); clf;
subplot(3,1,1);
plot(i,xx,'o',[1,nStep],[0,0])
ylabel('position')

subplot(3,1,2);
plot(i,vv,'o',[1,nStep],[0,0])
ylabel('velocity');

subplot(3,1,3);
plot(i,uu,'o',[1,nStep],[0,0])
ylabel('acceleration');