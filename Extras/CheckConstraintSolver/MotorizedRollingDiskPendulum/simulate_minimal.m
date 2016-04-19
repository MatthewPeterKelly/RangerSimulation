function [t,q0,q1,dq0,dq1,u,current, power,Fx,Fy,KE,PE,p0,pStar,p1] = simulate_minimal(P)

z0 = [P.q0;P.q1;P.dq0;P.dq1];

tSpan = P.tSpan;
options = odeset(...
    'RelTol',1e-12,...
    'AbsTol',1e-12);

dynFun = @(t,z)( rhs(z,P) );

sol = ode45(dynFun,tSpan,z0,options);

nSteps = ceil((tSpan(2)-tSpan(1))/P.dt);
t = linspace(tSpan(1), tSpan(2), nSteps);
z = deval(sol,t);
[p0,pStar,p1] = kinematics_minimal(z,P);

q0 = z(1,:);
q1 = z(2,:);
dq0 = z(3,:);
dq1 = z(4,:);

[u, current, power] = getMotorData(z,P);
[Fx, Fy] = forces_minimal(z,u,P);
[KE, PE] = energy_minimal(z,P);

end

function dz = rhs(z,P)

q0 = z(1,:);
q1 = z(2,:);
dq0 = z(3,:);
dq1 = z(4,:);

[Ir, Cp, Cd] = controller(q0,q1,dq0,dq1,P);

u = motorModel(q0,q1,dq0,dq1, Ir, Cp, Cd, P);

dz = dynamics_minimal(z,u,P);

end

function [torque, current, power] = getMotorData(z,P)

q0 = z(1,:);
q1 = z(2,:);
dq0 = z(3,:);
dq1 = z(4,:);

[Ir, Cp, Cd] = controller(q0,q1,dq0,dq1,P);

[torque, current, power] = motorModel(q0,q1,dq0,dq1, Ir, Cp, Cd, P);

end