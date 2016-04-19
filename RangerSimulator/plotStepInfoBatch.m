function plotStepInfoBatch(result)
%
% This function plots things that are computed once for each step
%

marker = {'rv','ko','b+'};

for iSlope=1:length(result)
    
    info = result(iSlope).info;
    nStep = length(info);
    
    idx = 1:nStep;
    dist = zeros(nStep,1);
    vel = zeros(nStep,1);
    time = zeros(nStep,1);
    cost = zeros(nStep,1);
    
    for iStep=1:nStep
        dist(iStep) = info(iStep).delPosX;
        vel(iStep) = info(iStep).meanVelX;
        time(iStep) = info(iStep).duration;
        cost(iStep) = info(iStep).CoT;
    end
    
    subplot(2,2,1); hold on;
    plot(idx,dist,marker{iSlope},'MarkerSize',10,'LineWidth',3);
    xlabel('step number')
    ylabel('position (m)')
    title('Step Length')
    
    subplot(2,2,2); hold on;
    plot(idx,time,marker{iSlope},'MarkerSize',10,'LineWidth',3);
    xlabel('step number')
    ylabel('duration (s)')
    title('Step Duration')
    
    subplot(2,2,3); hold on;
    plot(idx,vel,marker{iSlope},'MarkerSize',10,'LineWidth',3);
    xlabel('step number')
    ylabel('velocity (m/s)')
    title('Mean Velocity')
    
    subplot(2,2,4); hold on;
    plot(idx,cost,marker{iSlope},'MarkerSize',10,'LineWidth',3);
    xlabel('step number')
    ylabel('CoT')
    title('Cost of Transport')
    
end

% Write out the legend entries:
for i=1:length(result)
    if i==1
        legendText = ['legend(''slope = ' sprintf('%3.2f',180*result(i).slope/pi) ' deg'''];
    else
        legendText = [legendText, ', ''slope = ' sprintf('%3.2f',180*result(i).slope/pi) ' deg''']; %#ok<AGROW>
    end
end
legendText = [legendText, ');'];
subplot(2,2,2);
eval(legendText);
