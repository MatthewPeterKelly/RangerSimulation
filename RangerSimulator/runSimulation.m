function [Info, output] = runSimulation(nSteps, config, A, V)

%%%% Simulation of many steps:
idxDist = 1:length(config.model.dist.fx);
t0 = 0.0;
x0 = config.objFun.xLaunch;
for i=1:nSteps
    
    % Figure out which controller to run:
    if i==1 %Then approximate speed using initial state
        [~, dG] = getAbsoluteCoM(x0, config.model.dyn);
        v = dG(1);
    else % Approximate speed using previous step:
        v = info.meanVelX;
    end
    action = getAction(v,A,V);
    
    % Simulate one walking step
    if config.objFun.flagMex
        stepData(i) = oneStepSimPushDist_mex(t0, x0, action, config.model); %#ok<AGROW>
    else
        stepData(i) = oneStepSimPushDist(t0, x0, action, config.model); %#ok<AGROW>
    end
    
    % Compute useful data about that walking step
    info = getStepInfo(stepData(i), config.model);
    info.action = action;
    Info(i)= info;   %#ok<AGROW>
    
    if ~info.success  %If the step failed, then terminate simulation
        stepData = stepData(1:i);
        Info = Info(1:i);
        break;
    else
        t0 = stepData(i).t(:,end);
        x0 = stepData(i).x(:,end);
        idxDist = idxDist + length(stepData(i).t);
    end
end


% Combine all steps into a single large data structure for post-processing
if nargout == 2
output = mergeStructArray(stepData);
end

end