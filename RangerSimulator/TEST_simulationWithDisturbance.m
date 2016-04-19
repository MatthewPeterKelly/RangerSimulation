% MAIN_testController.m
%
% Simulate Ranger with the full controller running, and apply a push
% disturbance during the middle of the walk.
%


clc; clear;

%%%% Load parameters for Ranger %%%%
config.model = initializeModel();
config.model.sensors.Phi = config.model.dyn.Phi;
config.model.estimator = [];
config.model.control.data = getControlData();
config.model.control.data.dyn = config.model.dyn; %Assume perfect model


% How many steps to simulate?
nSteps = 15;

% A candidate walking controller  (not particularily well optimized)
A = [...
    0.05, 0.05, 0.0;   % Scissor Offset
    1.25, 1.2, 1.1;  % Scissor Gain
    0.7, 0.6, 0.2;   % ankle push
    0.27, 0.22, 0.25;  % critical step length
    0.06, 0.05, 0.05];   %Double stance delay


% Start walking from this state
xLaunch = [...  %Balanced at mid-stance
    0;
    1.0200;
    0;
    1.8000;
    -0.0050;
    0.0050;
    0;
    0;
    0;
    0;
    0;
    0];

config.objFun.xLaunch = xLaunch;
config.objFun.targetSpeed = 0.55;
config.objFun.speedKnots = [0, config.objFun.targetSpeed, 1.0];  % Knot points for linear interpolation
config.objFun.flagMex = true;

%%%% Construct a controller:
V = config.objFun.speedKnots;

%%% Construct a sample disturbance:
config.model.dist.t0 = 4.5;   %Start time for push
config.model.dist.t1 = 5.0;   % end time for push
config.model.dist.fx = 5.0;   % horizontal force (N)
config.model.dist.fy = 0.0;  % vertical force (N)

%%%% THIS IS THE IMPORTANT LINE:
[Info, output] = runSimulation(nSteps, config, A, V);


%% %% Analysis:

% Animation:
Animation.plotFunc = @(t,x)( drawRangerFancy(t,x,config.model.dyn) );
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
plotRobotMotors(output,config.model);

figure(105); clf;
plotStepInfo(Info);

figure(101); clf;
plotFullState(output);

figure(102); clf;
plotRobotState(output,config.model);

figure(105);  %Show on top
