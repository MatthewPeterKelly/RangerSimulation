function output = oneStepSim(x0, model)
% output = oneStepSim(tSpan, x0, model)
%
% Runs a simulation of Ranger for the desired time span (tSpan), starting
% from the state x0, using the controller in ctrlFun, and the physical
% parameters (param) for the robot, simulator, and environment.
%
% INPUT:
%   tSpan = [start, end] times
%   x0 = initial state (see notes for details)
%   ctrlFun = function handle for controller (see notes)
%   param = parameter struct
%
% OUTPUT:
%   output = struct with all simulation information
%
% NOTES:
% state = [z;dz] state vector
%   z = [6, 1] = full configuration vector
%   dz = [6, 1] = full configuration rate vector
%       (1) = hip horizontal position
%       (2) = hip vertical position
%       (3) = outer foot absolute angle
%       (4) = inner foot absolute angle
%       (5) = outer leg absolute angle
%       (6) = inner leg absolute angle
%
% input = u = [3, 1] = motor torque vector
% 	(1) = outer ankle joint torque (+leg, -foot)
% 	(2) = inner ankle joint torque (+leg, -foot)
% 	(3) = hip torque (+inner, -outer)
%

%   controller:
%       u = ctrlFun(x)
%           x = state vector
%           u = control vector
%
% Exit codes:
%
%   0 = continue! (not fallen, next step not reached)
%   1 = next step reached
%   2 = robot fell down
%

dt = model.dyn.dt;
nTimeStep = model.dyn.nDataPerStep;

% Allocate memory for the simulation:
x = zeros(12,nTimeStep+1);  %Full state
f = zeros(4,nTimeStep); % contact forces
c = zeros(4,nTimeStep); % contact locations
u = zeros(3,nTimeStep); % motor torques
k = false(2,nTimeStep); % Contact boolean
y = zeros(10,nTimeStep); % Sensor data
est = zeros(10,nTimeStep); % Estimator data
iRef = zeros(3,nTimeStep);
cp = zeros(3,nTimeStep);
cd = zeros(3,nTimeStep);
uCmd = zeros(3,nTimeStep);
qRef = zeros(3,nTimeStep);  %Target angles for the controller
dqRef = zeros(3,nTimeStep);  %Target angles for the controller
uRef = zeros(3,nTimeStep);   %Target motor torque
kp  = zeros(3,nTimeStep);   %torque proportional gain
kd = zeros(3,nTimeStep);   %torque derivative gain
fsmMode = zeros(1,nTimeStep);   %Enum for finite state machine state
power = zeros(3,nTimeStep);
current = zeros(3,nTimeStep);
uIdeal = zeros(3,nTimeStep);   %Desired torque, before complicated motor model stuff

% For now, assume no disturbance
dist = [0;0];

% Run the simulation:
x(:,1) = x0;
t = linspace(0,nTimeStep*dt,nTimeStep);
exitFlag = 3;   %Assume that we reach the end time of the simulation
dataLength = nTimeStep;
for i=1:nTimeStep
    
    if i == 1,
        y(:,i) = sensors(x(:,i), [0;0], model.sensors);
        currentLast = current(:,i);
    else
        y(:,i) = sensors(x(:,i), k(:,i-1), model.sensors);
        currentLast = current(:,i-1);
    end
    est(:,i) = estimator(y(:,i), model.estimator);
    
    [uRef(:,i), kp(:,i), kd(:,i), qRef(:,i), dqRef(:,i), fsmMode(i)] = ...
        walkControl(t(i), est(:,i), currentLast, model.control.data);
    
    [iRef(:,i), cp(:,i), cd(:,i), uIdeal(:,i)] = ...
        motorControl(est(:,i), uRef(:,i), kp(:,i), kd(:,i), qRef(:,i), dqRef(:,i), model.control);
    
    [x(:,i+1),f(:,i),c(:,i),k(:,i),u(:,i), power(:,i), current(:,i)] = simulate(x(:,i), iRef(:,i), cp(:,i), cd(:,i), model.dyn, model.motor, dist);
%             [x(:,i+1),f(:,i),c(:,i),k(:,i),u(:,i), power(:,i), current(:,i)] = simulate_mex(x(:,i), iRef(:,i), cp(:,i), cd(:,i), model.dyn, model.motor, dist);
    
    
    if checkFall(x(:,i+1), model.dyn);
        exitFlag = 2;   %The robot fell down!
        dataLength = i;
        break
    end
    
    % Mid-stance event detection:
    if  t(i) > model.dyn.minStepDuration
        if xor(k(1,i),k(2,i))  %Then in single stance
            if k(1,i)    % Outer stance
                thLast = x(5,i);
                thNext = x(5,i+1);
            else   % Inner stance
                thLast = x(6,i);
                thNext = x(6,i+1);
            end
            if (thLast > 0) && (thNext <= 0)   %Then we are at mid-stance
                exitFlag = 1;
                dataLength = i;
                break;
            end
        end
    end
    
end


% Pack up the results:
output.t = t(:,1:dataLength);
output.x = x(:,1:dataLength);
output.u = u(:,1:dataLength);
output.f = f(:,1:dataLength);
output.c = c(:,1:dataLength);
output.k = k(:,1:dataLength);
output.y = y(:,1:dataLength);
output.est = est(:,1:dataLength);
output.uCmd = uCmd(:,1:dataLength);
output.qRef = qRef(:,1:dataLength);
output.dqRef = dqRef(:,1:dataLength);
output.fsmMode = fsmMode(:,1:dataLength);
output.kp = kp(:,1:dataLength);
output.kd = kd(:,1:dataLength);
output.uRef = uRef(:,1:dataLength);
output.iRef = iRef(:,1:dataLength);
output.cp = cp(:,1:dataLength);
output.cd = cd(:,1:dataLength);
output.power = power(:,1:dataLength);
output.current = current(:,1:dataLength);
output.uIdeal = uIdeal(:,1:dataLength);
output.exitFlag = exitFlag;
output.dist = zeros(2,dataLength);  % No disturbances here!

end