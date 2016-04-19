function [t,q0,q1,dq0,dq1,u,current, power, Fx,Fy,KE,PE,p0,pStar,p1] = simulate_sym2(P)

% Get consistent initial conditions for the disk position and velocity
[p0,~,~,dp0] = autoGen_kinematics_minimal(P.q0,P.q1,P.dq1,P.dq0,P.r,P.a,P.b,P.slope);

%%%% HACK %%%%
% P.dt = 2*P.dt;

% Set up time stepping:
tSpan = P.tSpan;
nSteps = ceil((tSpan(2)-tSpan(1))/P.dt);
t = linspace(tSpan(1),tSpan(2),nSteps+1);
z0 = [p0;P.q0;P.q1];
dz0 = [dp0;P.dq0;P.dq1];
z = zeros(4,nSteps+1); z(:,1) = z0;
dz = zeros(4,nSteps+1); dz(:,1) = dz0;

for i=1:nSteps    % symplectic 2nd order step step
     [z(:,i+1), dz(:,i+1)] = sym2Step(z(:,i),dz(:,i),P);
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

function [zNext, dzNext] = sym2Step(z,dz,P)
% Second Order Symplectic Integrator (Verlet)
%	http://en.wikipedia.org/wiki/Verlet_integration
h = P.dt;

ddz1 = dynFun(z,dz,P);
z1 = z + h*dz + 0.5*ddz1*h*h;
dz1 = dz + h*ddz1;

ddz2 = dynFun(z1,dz1,P);
zNext = z1;
dzNext = dz1 + 0.5*h*(ddz1+ddz2);

end

function  ddz  = dynFun(z,dz,P)
    
    q0 = z(3,:);
    q1 = z(4,:);
    dq0 = dz(3,:);
    dq1 = dz(4,:);

    [Ir, Cp, Cd] = controller(q0,q1,dq0,dq1,P);
    u = motorModel(q0,q1,dq0,dq1, Ir, Cp, Cd, P);
    ddz = dynamics_contact(z,dz,u,P);
        
end


function [Fx,Fy,u,current,power] = getMetaData(z,dz,P)
    
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