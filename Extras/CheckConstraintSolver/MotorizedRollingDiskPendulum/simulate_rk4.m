function [t,q0,q1,dq0,dq1,u,current, power, Fx,Fy,KE,PE,p0,pStar,p1] = simulate_rk4(P)

% Get consistent initial conditions for the disk position and velocity
[p0,~,~,dp0] = autoGen_kinematics_minimal(P.q0,P.q1,P.dq1,P.dq0,P.r,P.a,P.b,P.slope);

%%%% HACK %%%%
% P.dt = 4*P.dt;

% Set up time stepping:
tSpan = P.tSpan;
nSteps = ceil((tSpan(2)-tSpan(1))/P.dt);
t = linspace(tSpan(1),tSpan(2),nSteps+1);
z0 = [p0;P.q0;P.q1];
dz0 = [dp0;P.dq0;P.dq1];
z = zeros(4,nSteps+1); z(:,1) = z0;
dz = zeros(4,nSteps+1); dz(:,1) = dz0;

for i=1:nSteps    % Rk4 step
     [z(:,i+1), dz(:,i+1)] = rk4Step(z(:,i),dz(:,i),P);
end

% Compute the other useful data:
[Fx,Fy,u,current,power] = getMetaData(z,dz,P);

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

function [zNext, dzNext] = rk4Step(z,dz,P)

Z = [z;dz];
h = P.dt;

k1 = dynFun(Z, P);
k2 = dynFun(Z + 0.5*h*k1, P);
k3 = dynFun(Z + 0.5*h*k2, P);
k4 = dynFun(Z + h*k3, P);

Znext = Z + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

zNext = Znext(1:4,:);
dzNext = Znext(5:8,:);

end

function  dZ  = dynFun(Z,P)
% First-order form of the dynamics
    
    q0 = Z(3,:);
    q1 = Z(4,:);
    dq0 = Z(7,:);
    dq1 = Z(8,:);
    
    z = Z(1:4,:);
    dz = Z(5:8,:);

    [Ir, Cp, Cd] = controller(q0,q1,dq0,dq1,P);
    u = motorModel(q0,q1,dq0,dq1, Ir, Cp, Cd, P);
    ddz = dynamics_contact(z,dz,u,P);
    
    dZ = [dz;ddz];
    
end


function [Fx,Fy,u,current,power] = getMetaData(z,dz,P)
% First-order form of the dynamics
    
    q0 = z(3,:);
    q1 = z(4,:);
    dq0 = dz(3,:);
    dq1 = dz(4,:);

    [Ir, Cp, Cd] = controller(q0,q1,dq0,dq1,P);
    [u, current, power] = motorModel(q0,q1,dq0,dq1, Ir, Cp, Cd, P);
    
    nGrid = length(u);
    F = zeros(2,nGrid);
    for i=1:nGrid
       [~, F(:,i)] = dynamics_contact(z(:,i),dz(:,i),u(i),P); 
    end
    
    Fx = F(1,:);
    Fy = F(2,:);
    
end