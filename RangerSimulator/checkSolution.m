function result = checkSolution(soln, config, model)
% result = checkSolution(soln, config, model)
%
% Run three simulations, and return the data for plotting


% Update controller
model.control.data.gains = soln.gains;
model.control.data.param = soln.param;

% Store the list of slopes to try:
slope = soln.scale*config.maxSlope*[-1,0,1];

% Allocate memory for data logging:
nSlope = length(slope);
nSteps = config.nSteps;
for iSlope = 1:nSlope
    model.dyn.ground = [0, slope(iSlope), 0,0,0];  %Planar sloping ground
    x0 = config.x0;
    for iStep=1:nSteps 
        % stepData = oneStepSim(x0, model);  
        stepData = oneStepSim_mex(x0, model);  
        
        info = getStepInfo(stepData, model); 
        if ~info.success  %If the step failed, then terminate simulation
            break;
        else
            result(iSlope).info(iStep) = info;   %#ok<AGROW>
            result(iSlope).stepData(iStep) = stepData;   %#ok<AGROW>
            x0 = stepData.x(:,end);
        end
    end
    result(iSlope).slope = slope(iSlope);
end

end