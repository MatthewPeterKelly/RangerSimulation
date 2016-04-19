% TEST_SimpleWalk.m
%
% This script tests a simple walking gait, starting from balanced at
% mid-stance. It uses a single set of gains for the walking controller,
% rather than adapting the while walking.
%
%#ok<*SAGROW>
%#ok<*UNRCH>

clc; clear;

model = initializeModel();

flagMex = false;


%%%% Controller Parameters, Identical to real robot: %%%%%%%%%%%%%%%%%%%%%%
model.control.data.walk.hip_kp = 25;
model.control.data.walk.hip_kd = 3;
model.control.data.walk.ank_push_kp = 15;
model.control.data.walk.ank_push_kd = 2.5;
model.control.data.walk.ank_stance_kp = 15;
model.control.data.walk.ank_stance_kd = 1;
model.control.data.walk.ank_swing_kp = 20;
model.control.data.walk.ank_swing_kd = 1;

model.control.data.walk.ank_push = 0.7;  %Normalized push-off angle
model.control.data.walk.crit_step_length = 0.2;
model.control.data.walk.scissor_gain = 1.25;
model.control.data.walk.scissor_offset = 0.05;
model.control.data.walk.doubleStance_duration = 0.04;   %How long to continue push-off after heel-strike
%
model.control.data.walk.ank_flipTarget = 0.2;   % relative ankle angle when foot is flipped up
model.control.data.walk.ank_holdLevel = 0.0;   %absolute foot angle target for hold level
model.control.data.walk.ank_pushTarget = -1.0;  %absolute foot angle target for push-off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




model.control.data.dyn = model.dyn; %Assume perfect model

nSteps = 2;

x0 = 0;
y0 = 0;
phi0 = 0;
phi1 = model.dyn.Phi;
th0 = -0.005;
th1 = 0.005;

dx0 = 0;
dy0 = 0;
dphi0 = 0;
dphi1 = 0;
dth0 = 0;
dth1 = 0;

x0 = [...
    x0; y0; phi0; phi1; th0; th1;
    dx0; dy0; dphi0;dphi1;dth0; dth1];

% Project the robot up to the ground
x0(1:6,1) = fixInitialState(x0(1:6,1),model.dyn);

tic
for i=1:nSteps
    
    if flagMex
        try
            stepData(i) = oneStepSim_mex(x0, model);
        catch
            disp('Attempting to build simulator...')
            try
                coder -build oneStepSim.prj
                stepData(i) = oneStepSim_mex(x0, model);
            catch
                error('Failed to build simulator. ');
                disp('You can still run it (slowly) in native matlab by setting flagMex = false.');
            end
        end
    else
        stepData(i) = oneStepSim(x0, model);
    end
    
    info(i) = getStepInfo(stepData(i), model);
    
    if ~info(i).success  %If the step failed, then terminate simulation
        stepData = stepData(1:i);
        info = info(1:i);
        break;
    else
        x0 = stepData(i).x(:,end);
    end
end
toc


% Combine all steps into a single large data structure for post-processing
output = mergeStructArray(stepData);

%% %% Analysis:

% Animation:
Animation.plotFunc = @(t,x)( drawRangerFancy(t,x,model.dyn) );
Animation.speed = 0.25;
Animation.figNum = 100;
Animation.verbose = true;
camera = getCameraTracking(output);
fullState = [output.x; output.u; camera; output.dist];
animate(output.t, fullState, Animation);

%%%% Keyboard commands during animation:
%         'space' - toggle pause
%         'r' - reset animation
%         'uparrow' - go faster
%         'downarrow' - go slower
%         'rightarrow' - jump forward by 5 frames
%         'leftarrow' - jump backward by 5 frames
%         'esc' - quit animation
%%%%

%%

figure(103); clf;
plotContactInfo(output);

figure(104); clf;
plotRobotMotors(output,model);

figure(105); clf;
plotStepInfo(info);

figure(101); clf;
plotFullState(output);

figure(102); clf;
plotRobotState(output,model);

figure(105);  %Show on top



