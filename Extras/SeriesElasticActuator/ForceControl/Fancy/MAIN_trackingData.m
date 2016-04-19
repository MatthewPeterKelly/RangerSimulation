% MAIN - runs simulation of the force controller
%
%

% Motor parameters:
P.Ga = 34; % Gear ratio for the ankle joint
P.Km = 0.018; %  Motor torque constant
P.Jm = 1.6e-6;  % motor rotor inertia
P.C1 = 0.0; % Motor viscous damping
P.C0 = 0.01; % Motor constant friction
P.Mu = 0.1;  % load dependent friction
P.Smooth = 0.1; %  Smoothing parameter for Sgn() and Abs() functions
P.Imax = 8.0;  % Maximum current allowed by the motor controllers
P.Ks = 14; %  Effective compliance of the cable between motor and stance foot

% Force Controller parameters:
P.wn = 30;   % Motor controller tracking frequency (Hz)
P.xi = 1.0;   % Motor controller damping ratio

Data = load('TrackingData.mat');

% Test paramters:
P.forceProfile = 'forceData';  %{'freqSweep','squareWave','constant'};
P.angleProfile = 'angleData'; %{'sineWave','fastSineWave','constant'}
P.data.time = Data.t;
P.data.force = Data.u0;
P.data.angle = Data.q0;
P.data.rate = Data.dq0;

% Run simulation:
tSpan = P.data.time([1,end]);
z0 = [P.data.angle(1); P.data.rate(1)];
options = odeset(...
    'AbsTol',1e-6,...
    'RelTol',1e-6);
dynFun = @(t,z)(  motorSystem(t,z,P)  );
sol = ode45(dynFun,tSpan,z0,options);

% Extract solution:
t = linspace(tSpan(1), tSpan(2), 400);
z = deval(sol,t);
[dz, I, f, fRef, qEnd,dqEnd] = motorSystem(t,z,P);
q = z(1,:);
dq = z(2,:);
ddq = dz(2,:);

% Plot solution:
figure(50); clf;

subplot(2,2,1); hold on;
plot(t,q);
plot(t,qEnd);
xlabel('time')
ylabel('angle')

subplot(2,2,3); hold on;
plot(t,dq);
plot(t,dqEnd);
xlabel('time')
ylabel('rate')

subplot(2,2,2);
plot(t,I);
xlabel('time')
ylabel('current')

subplot(2,2,4); hold on;
plot(t,f);
plot(t,fRef,'k--');
xlabel('time')
ylabel('force')


