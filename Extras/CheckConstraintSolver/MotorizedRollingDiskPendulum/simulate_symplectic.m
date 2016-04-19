function [t,q0,q1,dq0,dq1,u,current, power,Fx,Fy,KE,PE,p0,pStar,p1] = simulate_symplectic(P)

z0 = [P.q0;P.q1];
dz0 = [P.dq0;P.dq1];

nSteps = ceil((P.tSpan(2)-P.tSpan(1))/P.dt);
[t,z,dz] = symplecticSim(z0, dz0, nSteps, P);

[p0,pStar,p1] = kinematics_minimal([z;dz],P);

q0 = z(1,:);
q1 = z(2,:);
dq0 = dz(1,:);
dq1 = dz(2,:);

[u, current, power] = getMotorData(z,dz,P);
[Fx, Fy] = forces_minimal([z;dz],u,P);
[KE, PE] = energy_minimal([z;dz],P);

end

function ddz = dynFun(z,dz,P)

q0 = z(1,:);
q1 = z(2,:);
dq0 = dz(1,:);
dq1 = dz(2,:);

[Ir, Cp, Cd] = controller(q0,q1,dq0,dq1,P);

u = motorModel(q0,q1,dq0,dq1, Ir, Cp, Cd, P);

ddz = dynamics_symplectic(z,dz,u,P);

end

function [torque, current, power] = getMotorData(z,dz,P)

q0 = z(1,:);
q1 = z(2,:);
dq0 = dz(1,:);
dq1 = dz(2,:);

[Ir, Cp, Cd] = controller(q0,q1,dq0,dq1,P);

[torque, current, power] = motorModel(q0,q1,dq0,dq1, Ir, Cp, Cd, P);

end


function [t,z,dz] = symplecticSim(z0, dz0, nSteps, P)

h = P.dt;
t = zeros(1,nSteps+1);
z = zeros(length(z0),nSteps+1);
dz = zeros(length(dz0),nSteps+1);

z(:,1) = z0;
dz(:,1) = dz0;

for i=1:nSteps
    t(i+1) = t(i) + h;
    dz(:, i+1) = dz(:, i) + h*dynFun(z(:,i),dz(:,i),P);
    z(:,i+1) = z(:,i) + h*dz(:, i+1);
end


end


function ddz = dynamics_symplectic(z,dz,u,p)

q0 = z(1,:);
q1 = z(2,:);
dq0 = dz(1,:);
dq1 = dz(2,:);

[ddq0,ddq1] = autoGen_dynamics_minimal(...
    q0,q1,dq1,dq0,...
    u,...
    p.m0,p.m1,p.g,p.r,p.a,p.b,p.I0,p.I1,p.slope);

ddz = [ddq0;ddq1];

end

