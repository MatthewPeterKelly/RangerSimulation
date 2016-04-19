% TEST_SimpleWalk.m
%
% This script tests a simple walking gait, starting from balanced at
% mid-stance.
%

clc; clear;


model = initializeModel();

model.dyn.ground = zeros(1,6);  %Flat ground

% % maxSlope = (pi/180)*1;
% % maxHeight = 0.01;
% % a = 0.5*maxHeight;
% % b = maxSlope/(2*pi*a);
% % c = 0.25;  %Start on top of the hill
% % model.dyn.ground = [0, 0, a,b,c,0];   %small rolling hills
% % model.dyn.ground = [0, 0.005, 0, 0, 0, 0];   %Slight slope
% % 


% model.dyn.ground = [0,0, 0.01, 1/3, 1/12,0];   %Wavy Ground


% model.dyn.ground = [0, 0.01, 0, 0, 0, 0];

model.sensors.Phi = model.dyn.Phi;

model.estimator = [];



%%%% Controller Parameters, Identical to real robot: %%%%%%%%%%%%%%%%%%%%%%
model.control.data.walk.hip_kp = 25;
model.control.data.walk.hip_kd = 3;
model.control.data.walk.ank_push_kp = 15;   
model.control.data.walk.ank_push_kd = 2.5;   
model.control.data.walk.ank_stance_kp = 15;
model.control.data.walk.ank_stance_kd = 1;
model.control.data.walk.ank_swing_kp = 20;
model.control.data.walk.ank_swing_kd = 1;

model.control.data.walk.ank_push = 0.8;  %Normalized push-off angle
model.control.data.walk.crit_step_length = 0.2;
model.control.data.walk.scissor_gain = 1.2;
model.control.data.walk.scissor_offset = 0.1;
model.control.data.walk.doubleStance_duration = 0.06;   %How long to continue push-off after heel-strike
%
model.control.data.walk.ank_flipTarget = 0.2;   % relative ankle angle when foot is flipped up
model.control.data.walk.ank_holdLevel = 0.0;   %absolute foot angle target for hold level
model.control.data.walk.ank_pushTarget = -1.0;  %absolute foot angle target for push-off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




model.control.data.dyn = model.dyn; %Assume perfect model

nSteps = 15;

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
    
%         stepData(i) = oneStepSim(x0, model); %#ok<SAGROW>
    stepData(i) = oneStepSim_mex(x0, model);  %#ok<SAGROW>
    
    info(i) = getStepInfo(stepData(i), model);  %#ok<SAGROW>
    
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

%%%% Analysis:

% Animation:
Animation.plotFunc = @(t,x)( drawRangerFancy(t,x,model.dyn) );
Animation.speed = 0.25;
Animation.figNum = 100;
Animation.verbose = true;
camera = getCameraTracking(output);
fullState = [output.x; output.u; camera; output.dist];
animate(output.t, fullState, Animation);

% figure(103); clf;
% plotContactInfo(output);

figure(104); clf;
plotRobotMotors(output,model);
% 
figure(105); clf;
plotStepInfo(info);
% 
% figure(101); clf;
% plotFullState(output);
% 
% figure(102); clf;
% plotRobotState(output,model);

figure(105);  %Show on top



%% Save Animation to file:
Animation.fileName = 'RangerPeriodWalk';
Animation.frameRate = 15;  %30;
Animation.speed = 1.0;
Animation.mode = 'gif';  %  must be 'mp4'  or 'gif'
tSpan = [2.758, 4.348];  %[0,20];
if output.t(1) > tSpan(1) || output.t(end) < tSpan(2)
    error('Invalid time span for video writing!')
end
time = tSpan(1):(Animation.speed/Animation.frameRate):tSpan(end);
state = interp1(output.t',fullState',time')';
saveAnimation(time, state, Animation);


