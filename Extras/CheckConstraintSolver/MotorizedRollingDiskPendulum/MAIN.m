% MAIN - Contact Simulation - Sanity check on rolling solver:
%
% Runs the simulation first with the minimal coordinates solution, and then


clc; clear;

initializeProblem;  %Store parameters in struct P

figure(103); clf;


% Minimal coordinates, ode45
[t,q0,q1,dq0,dq1,u, current, power, Fx,Fy,KE,PE] = simulate_minimal(P);
plotSoln(t,q0,q1,dq0,dq1,u, current, power,  Fx,Fy,KE,PE,'k-');

% Full time-stepping contact sim
[t,q0,q1,dq0,dq1,u, current, power, Fx,Fy,KE,PE] = simulate_contact(P);
plotSoln(t,q0,q1,dq0,dq1,u, current, power, Fx,Fy,KE,PE,'bx');

% Time stepping, rk4
[t,q0,q1,dq0,dq1,u,current, power,Fx,Fy,KE,PE] = simulate_rk4(P);
plotSoln(t,q0,q1,dq0,dq1,u, current, power,  Fx,Fy,KE,PE,'ro');

% Time stepping, sym2
[t,q0,q1,dq0,dq1,u,current, power,Fx,Fy,KE,PE] = simulate_sym2(P);
plotSoln(t,q0,q1,dq0,dq1,u, current, power,  Fx,Fy,KE,PE,'c^');

% Minimal coordinates, sympelctic euler
[t,q0,q1,dq0,dq1,u,current, power,Fx,Fy,KE,PE,p0,pStar,p1] = simulate_symplectic(P);
plotSoln(t,q0,q1,dq0,dq1,u, current, power,  Fx,Fy,KE,PE,'gs');



subplot(3,3,3);
legend('ode','dt','sym2','rk4','sym');