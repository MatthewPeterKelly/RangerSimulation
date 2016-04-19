function [t,q0,q1,dq0,dq1,u,Fx,Fy,KE,PE,p0,pStar,p1] = simulate_minimal(P)

z0 = [P.q0;P.q1;P.dq0;P.dq1];

tSpan = P.tSpan;
options = odeset(...
    'RelTol',1e-12,...
    'AbsTol',1e-12);

ctrl = @(z)( controller(z(1),z(3),P.kp,P.kd) );
dynFun = @(t,z)( dynamics_minimal(z,ctrl(z),P) );

sol = ode45(dynFun,tSpan,z0,options);

t = linspace(tSpan(1), tSpan(2), 2000);
z = deval(sol,t);
[p0,pStar,p1] = kinematics_minimal(z,P);

q0 = z(1,:);
q1 = z(2,:);
dq0 = z(3,:);
dq1 = z(4,:);

u = controller(q0,dq0,P.kp,P.kd);
[Fx, Fy] = forces_minimal(z,u,P);
[KE, PE] = energy_minimal(z,P);

end