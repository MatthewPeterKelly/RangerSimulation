function traj = getTraj_ds(config)

auxdata = config;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Set up the bounds on state and actuation                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% STATE:  [z,dz]
% CONTROL: u = [uHip, u1, u2];

% Configuration: z = [qf1, qf2, ql1, ql2];
% Rates: dz = [dqf1, dqf2, dql1, dql2];

p = config.dyn;  %Physical paramters

qf.low = -p.angleToe;
qf.upp = -p.angleHeel;
qf.flip = config.step.foot.flipMax;

ql.low = -config.step.leg.maxAngle;
ql.upp = config.step.leg.maxAngle;

%%%% Initial State:
%
% This initial state is important - there are four degrees of freedom in
% the state, but in double stance there are only three. This means that the
% state is over-constrained. The dynamics will project the constraints
% properly, but the initial state should be consistent. If desired, project
% the state using autoGen_projectConstraints.m
ql0 = config.step.init.stanceLegAngle;
qEps = 1e-8;
zLowInit = [0,0, ql0, -ql0];
zUppInit = [0,0, ql0, -ql0];
dzInit = zeros(1,4);  %No motion to start!

zLow = [qf.low, qf.low, qEps, ql.low];
zUpp = [qf.upp, qf.upp, ql.upp, -qEps];

% For now, no bounds on rates:
dzLow = -inf(1,4);
dzUpp = inf(1,4);

% Also no bounds on inputs:
uLow = -inf(1,3);
uUpp = inf(1,3);

%%%% DOUBLE STANCE %%%%
bounds.phase(1).initialtime.lower = 0;
bounds.phase(1).initialtime.upper = 0;
bounds.phase(1).finaltime.lower = config.step.bndDuration(1);
bounds.phase(1).finaltime.upper = config.step.bndDuration(2);
bounds.phase(1).initialstate.lower = [zLowInit, dzInit];
bounds.phase(1).initialstate.upper = [zUppInit, dzInit];
bounds.phase(1).state.lower = [zLow, dzLow];
bounds.phase(1).state.upper = [zUpp, dzUpp];
bounds.phase(1).finalstate.lower = [zLow, dzLow];
bounds.phase(1).finalstate.upper = [zUpp, dzUpp];
bounds.phase(1).control.lower = uLow;
bounds.phase(1).control.upper = uUpp;
bounds.phase(1).integral.lower = 0;
bounds.phase(1).integral.upper = inf;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Initialization                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

if strcmp(config.opt.loadGuess,'')  %Then simple guess

tGuess = [0;mean(config.step.bndDuration)];

qf1 = [0;0];  
qf2 = [0;0]; 
ql1 = [ql0; ql0];
ql2 = -[ql0;ql0];
zGuess = [qf1,qf2,ql1,ql2];

dqf1 = [0;0];
dqf2 = [0;0];
dql1 = (diff(ql1)/diff(tGuess))*[1;1];
dql2 = (diff(ql2)/diff(tGuess))*[1;1];
dzGuess = [dqf1,dqf2,dql1,dql2];

u1 = [0;0];
uHip = [0;0];
u2 = [0;0];
uGuess = [u1,uHip,u2];

else  %Load guess from file
    Data = load(config.opt.loadGuess);
    tGuess = Data.traj.phase.t';
    zGuess = Data.traj.phase.z';
    dzGuess = Data.traj.phase.dz';
    uGuess = Data.traj.phase.u';    
end

guess.phase(1).time = tGuess;
guess.phase(1).state = [zGuess,dzGuess];
guess.phase(1).control = uGuess;
guess.phase(1).integral = trapz(tGuess,costFun(zGuess,dzGuess,uGuess,config.dyn));


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Constraints:                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% EVENT:

%Step length (horizontal distance traversed by the hip)
bounds.eventgroup(1).lower = config.step.hipDist;
bounds.eventgroup(1).upper = config.step.hipDist;

%%%% PATH:

% Contact forces must be positive:
bounds.phase(1).path.lower = zeros(1,2);
bounds.phase(1).path.upper = inf(1,2);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Set up for GPOPS                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


setup.name = 'rangerTraj_ds';
setup.functions.continuous = @continuous;
setup.functions.endpoint = @endpoint;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;

setup.nlp.solver = 'snopt'; % {'ipopt','snopt'}
setup.nlp.snoptoptions.tolerance = config.opt.nlp.tol;
setup.nlp.snoptoptions.maxiterations = config.opt.nlp.maxIter;

%GPOPS Defaults:
setup.displaylevel = 2;
setup.scales.method = 'none';
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'first';
setup.derivatives.dependencies = 'sparse';
setup.mesh.method = 'hp-PattersonRao';
setup.mesh.maxiterations = config.opt.mesh.maxIter; 
setup.mesh.tolerance = config.opt.mesh.tol;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 10;

% Configure the solution mesh
setup.mesh.phase(1).fraction = ones(1,config.opt.nSegment)/config.opt.nSegment;
setup.mesh.phase(1).colpoints = config.opt.nColPts*ones(1,config.opt.nSegment);

setup.method = 'RPM-Differentiation';  %{'RPM-Integration' 'RPM-Differentiation'};


%%%% THIS IS THE KEY LINE:
output = gpops2(setup);


%%%% Format the result:
traj = formatOutput_ds(output);

end


function cost = costFun(z,dz,u,p)
%
% z = configuration
% dz = dz/dt
% u = control torques
% f = contact forces
%

cost = sum(u.^2,2);

end


function output = endpoint(input)

p = input.auxdata.dyn;

z0 = input.phase(1).initialstate(1:4);
zF = input.phase(1).finalstate(1:4);

%Position of contact point at start and end of step
pHip0 = getJointPos_ds(z0',p);
pHipF = getJointPos_ds(zF',p);
hipHorizDist = pHipF(1) - pHip0(1);

output.eventgroup(1).event = hipHorizDist;  

output.objective = input.phase(1).integral;

end


function output = continuous(input)

p = input.auxdata.dyn;

z = input.phase(1).state(:,1:4);   %Configuration
dz = input.phase(1).state(:,5:8);   %Rates
u = input.phase(1).control(:,1:3);   %Actuators
[ddz, f] = dynamics_ds(z',dz',u',p); 
ddz = ddz'; f = f';

output(1).path = [f(:,2),f(:,4)];  %Positive contact forces
output(1).dynamics = [dz,ddz];
output(1).integrand = costFun(z,dz,u,p);

end




