%MAIN - series elastic actuator
%
% Runs a simulation of a simple pendulum that is controlled by a series
% elastic actuator
%


%%%% NOTES %%%%
%
% It seems like the best control strategy is to send a command torque that
% has three parts:
%
% 1) Adds critical damping between the motor and the end effector
% 
% 2) PD-controller torque for reference tracking
%
% 3) Compensation for the current state of the spring
%
%%%% ~~~~ %%%%


P.Jm = 0.0018;  %Motor inertia
P.Ks = 14;   %Spring between motor and output
P.Jf = 0.45;   %Pendulum inertia about shaft
P.Kf = 15;  %m*g*l = gravity torque constant
P.uMax = 5;   %Maximum motor torque   %  <--  This make it trickey

z0 = [...   %Initial state
    0.5;  %Pendulum angle
    0.5;  %Motor angle
    0.0;  %pendulum rate
    0.0];  %Motor rate

% "get out of the way" controller
wn = 50*2*pi;   %natural frequency
xi = 1.0;   %Damping ratio

kp = 15;
kd = 5;

md = 0.01;  %Damping between motor and actuator 

% ctrlFun = @(t,z)( 0 );   %Passive controller for now
% ctrlFun = @(t,z)( ctrlGetOutOfTheWay(t,z,xi,wn,P) );    %<--- WORKS WELL
% ctrlFun = @(t,z)( ones(size(t)) );   %constant torque for now
% ctrlFun = @(t,z)( 0.5*sin(4*t) );   %sine curve forcing
% ctrlFun = @(t,z)( ctrlTrackMotor(t,z,kp,kd) );
% ctrlFun = @(t,z)( ctrlTrackOutput(t,z,kp,kd) );
% ctrlFun = @(t,z)( ctrlTrackSmart(t,z,kp,kd,md,P) );
% ctrlFun = @(t,z)( ctrlTrackForce(t,z,kp,kd,P) );
% ctrlFun = @(t,z)( ctrlActiveTracking(t,z,xi,wn,P) );   %<--- WORKS WELL
% ctrlFun = @(t,z)( ctrlNoOscillation(t,z,xi,P) );
ctrlFun = @(t,z)( ctrlCriticalTracking(t,z,xi,kp,kd,P) );   %  <-- BEST SOLUTION


tSpan = [0,2];

options = odeset(...
    'RelTol',1e-6,...
    'AbsTol',1e-6);

dynFun = @(t,z)( dynamics(z,ctrlFun(t,z),P) );

sol = ode45(dynFun,tSpan,z0,options);

t = linspace(tSpan(1), tSpan(2), 500);
z = deval(sol,t);

qf = z(1,:);    % foot angle (end effector)
qm = z(2,:);     % motor angle 
dqf = z(3,:);
dqm = z(4,:);

Tm = ctrlFun(t,z);
Tf = -P.Ks*(qf-qm);

figure(1); clf;

subplot(3,1,1); hold on;
plot(t,qf)
plot(t,qm)
legend('qf','qm');
xlabel('time')
ylabel('angle')

subplot(3,1,2); hold on;
plot(t,dqf)
plot(t,dqm)
legend('dqf','dqm');
xlabel('time')
ylabel('rate')

subplot(3,1,3); hold on;
plot(t,Tf);
plot(t,Tm);
legend('Tf','Tm');
xlabel('time')
ylabel('torque')
