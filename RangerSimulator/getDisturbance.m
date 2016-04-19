function [t, fx, fy] = getDisturbance(dist)
% [t, fx, fy] = getDisturbance(dist)
% 
% dist.tSpan = [0,6];
% dist.dt = 0.001;
% dist.bnd.startTime = [0.1, 1.3];
% dist.bnd.duration = [0.2,0.8];
% dist.bnd.xImpulse = [10,50];
% dist.bnd.yImpulse = [1,5];
%

B = dist.bnd;
tStart = dist.tSpan(1);
tFinal = dist.tSpan(2);
dt = dist.dt;

t0 = B.startTime(1) + (B.startTime(2)-B.startTime(1))*rand(1);
T = B.duration(1) + (B.duration(2)-B.duration(1))*rand(1);
Jx = B.xImpulse(1) + (B.xImpulse(2)-B.xImpulse(1))*rand(1);
Jy = B.yImpulse(1) + (B.yImpulse(2)-B.yImpulse(1))*rand(1);

Fx = Jx/T;
Fy = Jy/T;
t1 = t0 + T;

t = tStart:dt:tFinal;
active = t0 < t & t < t1;

fx = zeros(size(t));
fx(active) = Fx;

fy = zeros(size(t));
fy(active) = Fy;

end