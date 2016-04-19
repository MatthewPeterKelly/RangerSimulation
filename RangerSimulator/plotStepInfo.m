function plotStepInfo(info)
%
% This function plots things that are computed once for each step
%

nStep = length(info);

idx = 1:nStep;
dist = zeros(nStep,1);
vel = zeros(nStep,1);
time = zeros(nStep,1);
cost = zeros(nStep,1);

for i=1:nStep
   dist(i) = info(i).delPosX;
   vel(i) = info(i).meanVelX;
   time(i) = info(i).duration;
    cost(i) = info(i).CoT;
end

subplot(2,2,1);
plot(idx,dist,'ko','MarkerSize',10,'LineWidth',3);
xlabel('step number')
ylabel('position (m)')
title('Step Length')

subplot(2,2,2);
plot(idx,time,'ko','MarkerSize',10,'LineWidth',3);
xlabel('step number')
ylabel('duration (s)')
title('Step Duration')

subplot(2,2,3);
plot(idx,vel,'ko','MarkerSize',10,'LineWidth',3);
xlabel('step number')
ylabel('velocity (m/s)')
title('Mean Velocity')

subplot(2,2,4);
plot(idx,cost,'ko','MarkerSize',10,'LineWidth',3);
xlabel('step number')
ylabel('CoT')
title('Cost of Transport')

end