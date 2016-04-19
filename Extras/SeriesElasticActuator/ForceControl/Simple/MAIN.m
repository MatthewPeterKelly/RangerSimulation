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

% Test paramters:
P.forceProfile = 'squareWave';  %{'freqSweep','squareWave'};

% Run simulation:
tSpan = [0,3];
z0 = [0;0];
options = odeset(...
    'AbsTol',1e-6,...
    'RelTol',1e-6);
dynFun = @(t,z)(  motorSystem(t,z,P)  );
sol = ode45(dynFun,tSpan,z0,options);

% Extract solution:
t = linspace(tSpan(1), tSpan(2), 400);
z = deval(sol,t);
[dz, I, f, fRef] = motorSystem(t,z,P);
q = z(1,:);
dq = z(2,:);
ddq = dz(2,:);

% Plot solution:
figure(5); clf;

subplot(2,2,1);
plot(t,q);
xlabel('time')
ylabel('angle')

subplot(2,2,3)
plot(t,dq);
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


