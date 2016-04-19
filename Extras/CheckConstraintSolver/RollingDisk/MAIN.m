% MAIN - Contact Simulation - Sanity check on rolling solver:
%
%

clc; clear;

P.m = 0.87623;
P.I = 0.26342;
P.g = 6.2743;
P.r = 0.253;
P.slope = (pi/180)*5;
P.dt = 0.01;

% Turns out that acceleration and contact forces are constant...
[ddq,Fx,Fy] = autoGen_dynamics_minimal(P.m,P.g,P.r,P.I,P.slope);

% Run a trivial simulation:
q0 = 0.0;
dq0 = 0.0;
[p0,dp0] = autoGen_kinematics(q0,dq0,ddq,P.r,P.slope);
z0 = [p0;q0];
dz0 = [dp0;dq0];

% Set up time stepping:
tSpan = [0,10];
nSteps = ceil((tSpan(2)-tSpan(1))/P.dt);
t = linspace(0,nSteps*P.dt,nSteps+1);
z = zeros(3,nSteps+1); z(:,1) = z0;
dz = zeros(3,nSteps+1); dz(:,1) = dz0;
F = zeros(2,nSteps+1); 
for i=1:nSteps
    %Symplectic Euler step:    
   [ddz, F(:,i)] = dynamics_contact(z(:,i),dz(:,i),P);
    dz(:,i+1) = dz(:,i) + P.dt*ddz;
    z(:,i+1) = z(:,i) + P.dt*dz(:,i+1);
end
F(:,end) = F(:,end-1);

% Analytic solution for minimal coordinates:
q = q0 + dq0*t + 0.5*ddq*t.^2;
dq = dq0 + ddq*t;
[p,dp] = autoGen_kinematics(q,dq,ddq,P.r,P.slope);
x = p(1,:);
y = p(2,:);
dx = dp(1,:);
dy = dp(2,:);

%%%% Plot the solution:
figure(1); clf;

subplot(2,4,1); hold on;
plot(t,q)
plot(t,z(3,:))
xlabel('t')
ylabel('q')

subplot(2,4,2); hold on;
plot(t,x)
plot(t,z(1,:))
xlabel('t')
ylabel('x')

subplot(2,4,3); hold on;
plot(t,y)
plot(t,z(2,:))
xlabel('t')
ylabel('y')

subplot(2,4,5); hold on;
plot(t,dq)
plot(t,dz(3,:))
xlabel('t')
ylabel('dq')

subplot(2,4,6); hold on;
plot(t,dx)
plot(t,dz(1,:))
xlabel('t')
ylabel('dx')

subplot(2,4,7); hold on;
plot(t,dy)
plot(t,dz(2,:))
xlabel('t')
ylabel('dy')

subplot(2,4,4); hold on;
plot(t,Fx*ones(size(t)))
plot(t,F(1,:));
xlabel('t')
ylabel('Fx')

subplot(2,4,8); hold on;
plot(t,Fy*ones(size(t)))
plot(t,F(2,:));
xlabel('t')
ylabel('Fy')
