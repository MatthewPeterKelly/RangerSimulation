function [dist, t] = getDisturbance(nStep,xAmp,yAmp,wn,model,flagNormal)
% [dist, t] = getDisturbance(nStep,xAmp,yAmp,wn,model,flagNormal)
%
% This function computes a disturbance that should be applied at the hips
%
% nStep = number of steps to generate data for
% xAmp = xAmp(t) = function handle for amplitude
% yAmp = yAmp(t) = function handle for amplidute
% wn = cut-off frequency for noise
% model = model struct for robot
% if flagNormal then use normal disturbution
% else then use uniform distribution
%

warning('THIS CODE IS DEPRECATED. USE THE VERSION IN CONTROLLER DESIGN.');

nSamples = nStep*model.dyn.nDataPerStep;
dt = model.dyn.dt;
duration = (nSamples-1)*dt;

t = linspace(0,duration,nSamples);

if flagNormal
    xRandn = -1 + 2*rand(1,nSamples);
    yRandn = -1 + 2*rand(1,nSamples);
else
    xRandn = randn(1,nSamples);
    yRandn = randn(1,nSamples);
end
[B,A] = butter(2,2*dt*wn);
xRandn = filtfilt(B,A,xRandn);
yRandn = filtfilt(B,A,yRandn);

xRandn = xAmp(t).*xRandn;
yRandn = yAmp(t).*yRandn;

dist = [xRandn; yRandn];

end