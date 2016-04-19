function traj = getTraj_ss(config)

auxdata = config;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Set up the bounds on state and actuation                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% STATE:  z = [ [qf1,qf2,ql1,ql2], [dqf1,dqf2,dql1,dql2] ]
% CONTROL: u = [u1, uHip, u2];

qToe = config.step.foot.toe;
qHeel = config.step.foot.heel;
qFlip = config.step.foot.flip;
qLeg = config.step.leg.maxAngle;

zLow = [qToe,qToe,-qLeg, -qLeg];
zUpp = [qHeel,qFlip,qLeg,qLeg];
dzLow = -inf(1,4);
dzUpp = inf(1,4); 
dzUpp(1) = 0;   %Stance foot cannot roll backward

zLow0 = zLow;  
zUpp0 = zUpp; 
zLowF = zLow; zLowF(2) = 0;   %Swing foot ends in neutral position
zUppF = zUpp; zUppF(2) = 0;

dzLow0 = dzLow;  
dzUpp0 = dzUpp; 
dzLowF = dzLow; dzLowF(2) = 0;   % Swing foot ends stationary
dzUppF = dzUpp; dzUppF(2) = 0;

bounds.phase.initialtime.lower = 0;
bounds.phase.initialtime.upper = 0;
bounds.phase.finaltime.lower = config.step.time;
bounds.phase.finaltime.upper = config.step.time;
bounds.phase.initialstate.lower = [zLow0, dzLow0];
bounds.phase.initialstate.upper = [zUpp0, dzUpp0];
bounds.phase.state.lower = [zLow, dzLow];
bounds.phase.state.upper = [zUpp, dzUpp];
bounds.phase.finalstate.lower = [zLowF, dzLowF];
bounds.phase.finalstate.upper = [zUppF, dzUppF];
bounds.phase.control.lower = -inf(1,3);
bounds.phase.control.upper = inf(1,3);
bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = inf;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Initialization                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


if strcmp('',config.opt.loadGuess) %Then simple initial guess
    tGuess = [0;config.step.time];
    
    qf1 = [0;0];
    qf2 = [0;0];
    ql1 = [0.2; -0.2];
    ql2 = [-0.2; 0.2];
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

guess.phase.time = tGuess;
guess.phase.state = [zGuess,dzGuess];
guess.phase.control = uGuess;
guess.phase.integral = trapz(tGuess,costFun(zGuess,dzGuess,uGuess));


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Constraints:                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Height of the foot at the start and end of the step
bounds.eventgroup(1).lower = 0;
bounds.eventgroup(1).upper = 0;

%Step length (horizontal distance traversed by the hip)
bounds.eventgroup(2).lower = config.step.length;
bounds.eventgroup(2).upper = config.step.length;

%Periodic constraint on configuration:
bounds.eventgroup(3).lower = zeros(1,4);
bounds.eventgroup(3).upper = zeros(1,4);

%Periodic constraint on rates:
bounds.eventgroup(4).lower = zeros(1,4);
bounds.eventgroup(4).upper = zeros(1,4);

%Ankle velocity at the end of the step:
bounds.eventgroup(5).lower = 0;
bounds.eventgroup(5).upper = 0;

%Swing ankle height:
bounds.phase.path.lower = 0.8*(config.dyn.r - config.dyn.d);
bounds.phase.path.upper = 2*(config.dyn.r - config.dyn.d);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Set up for GPOPS                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


setup.name = 'rangerTraj';
setup.functions.continuous = @continuous;
setup.functions.endpoint = @endpoint;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;

setup.nlp.solver = 'snopt'; % {'ipopt','snopt'}
setup.nlp.snoptoptions.tolerance = config.opt.nlp.tol;
setup.nlp.snoptoptions.maxiterations = config.opt.nlp.maxIter;

setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.derivatives.dependencies = 'sparse';
setup.mesh.method = 'hp-PattersonRao';
setup.mesh.maxiterations = config.opt.mesh.maxIter;
setup.mesh.tolerance = config.opt.mesh.tol;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 10;

% Configure the solution mesh
setup.mesh.phase.fraction = ones(1,config.opt.nSegment)/config.opt.nSegment;
setup.mesh.phase.colpoints = config.opt.nColPts*ones(1,config.opt.nSegment);
setup.mesh.phase.colpoints([1,end]) = 2*config.opt.nColPts;  %Extra points near transitions


setup.method = 'RPM-Differentiation';  %{'RPM-Integration', 'RPM-Differentiation'};


%%%% THIS IS THE KEY LINE:
output = gpops2(setup);


traj = formatOutput_ss(output);

end


function cost = costFun(z,~,u)
% Simple quadratic cost function

u1 = u(:,1);
uHip = u(:,2);
u2 = u(:,3);

qf2 = z(:,2);

cost = uHip.^2 + u1.^2 + u2.^2 + ...
    10*(qf2-pi/2).^2;   %Foot Flip

end


function output = endpoint(input)

p = input.auxdata.dyn;
z0 = input.phase.initialstate(:,1:4);
dz0 = input.phase.initialstate(:,5:8);
zF = input.phase.finalstate(:,1:4);
dzF = input.phase.finalstate(:,5:8);

%Position of contact point at start and end of step
p2c0 = contactKinematics(z0',dz0',p);

%Position of the hip at the start and end of the step
pHip0 = kinematics_ss(z0',dz0',p);
[pHipF,~,~,~,~,dp2F] = kinematics_ss(zF',dzF',p);

%Constrain the initial contact point to be on the ground:
output.eventgroup(1).event = p2c0(2);

%Step length constraint:
output.eventgroup(2).event = pHipF(1) - pHip0(1);

%Periodic constraint:
[za, dza] = heelStrike(zF',dzF',p); za = za'; dza = dza';
output.eventgroup(3).event = za-z0;
output.eventgroup(4).event = dza-dz0;

%Ankle velocity constraint:
output.eventgroup(5).event = dp2F(1);  %No horizontal speed at ankle

output.objective = input.phase.integral;

end


function output = continuous(input)

% t = input.phase.time;
z = input.phase.state(:,1:4);
dz = input.phase.state(:,5:8);
u = input.phase.control;
p = input.auxdata.dyn;
ddz =dynamics_ss(z',dz',u',p)';
[~,~,p2] = kinematics_ss(z',dz',p); p2 = p2';

output.path = p2(:,2);
output.dynamics = [dz,ddz];
output.integrand = costFun(z,dz,u);

end




