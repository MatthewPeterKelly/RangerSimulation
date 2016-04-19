function [t,q0,q1,dq0,dq1,u,current, power, Fx,Fy,KE,PE,p0,pStar,p1] = simulate_contact(P)

% Get consistent initial conditions for the disk position and velocity
[p0,~,~,dp0] = autoGen_kinematics_minimal(P.q0,P.q1,P.dq1,P.dq0,P.r,P.a,P.b,P.slope);

% Set up time stepping:
tSpan = P.tSpan;
nSteps = ceil((tSpan(2)-tSpan(1))/P.dt);
t = linspace(tSpan(1),tSpan(2),nSteps+1);
z0 = [p0;P.q0;P.q1];
dz0 = [dp0;P.dq0;P.dq1];
z = zeros(4,nSteps+1); z(:,1) = z0;
dz = zeros(4,nSteps+1); dz(:,1) = dz0;
F = zeros(2,nSteps+1); 
u = zeros(1,nSteps+1);
current = zeros(1,nSteps+1);
power = zeros(1,nSteps+1);

for i=1:nSteps
    %Symplectic Euler step:   
    [Ir, Cp, Cd] = controller(z(3,i),z(4,i), dz(3,i),dz(4,i),P);
    [u(i), current(i), power(i)] = motorModel(z(3,i),z(4,i), dz(3,i),dz(4,i), Ir, Cp, Cd, P);
    [ddz, F(:,i)] = dynamics_contact(z(:,i),dz(:,i),u(i),P);
    dz(:,i+1) = dz(:,i) + P.dt*ddz;
    z(:,i+1) = z(:,i) + P.dt*dz(:,i+1);
end
F(:,end) = F(:,end-1);
Fx = F(1,:);
Fy = F(2,:);
[Ir, Cp, Cd] = controller(z(3,end),z(4,end), dz(3,end),dz(4,end),P);
[u(end), current(end), power(end)] = motorModel(z(3,end),z(4,end), dz(3,end),dz(4,end), Ir, Cp, Cd, P);


% Compute max energy error:
[KE,PE] = energy_contact(z,dz,P);

% Kinematics of the key points
[p0,pStar,p1] = kinematics_contact(z,dz,P);

% Unpack:
q0 = z(3,:);
q1 = z(4,:);
dq0 = dz(3,:);
dq1 = dz(4,:);

end