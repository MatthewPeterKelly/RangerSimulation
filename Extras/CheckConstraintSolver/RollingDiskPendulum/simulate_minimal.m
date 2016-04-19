function [t,q0,q1,dq0,dq1,Fx,Fy,KE,PE,p0,pStar,p1] = simulate_minimal(P)

z0 = [P.q0;P.q1;P.dq0;P.dq1];

tSpan = P.tSpan;
options = odeset(...
    'RelTol',1e-12,...
    'AbsTol',1e-12);

dynFun = @(t,z)( dynamics_minimal(z,P) );

sol = ode45(dynFun,tSpan,z0,options);

t = linspace(tSpan(1), tSpan(2), 250);
z = deval(sol,t);
[p0,pStar,p1] = kinematics_minimal(z,P);
[Fx, Fy] = forces_minimal(z,P);
[KE, PE] = energy_minimal(z,P);

q0 = z(1,:);
q1 = z(2,:);
dq0 = z(3,:);
dq1 = z(4,:);

end