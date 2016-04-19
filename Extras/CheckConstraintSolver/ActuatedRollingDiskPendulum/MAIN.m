% MAIN - Contact Simulation - Sanity check on rolling solver:
%
% Runs the simulation first with the minimal coordinates solution, and then


clc; clear;

initializeProblem;  %Store parameters in struct P

figure(103); clf;

[t,q0,q1,dq0,dq1,u,Fx,Fy,KE,PE] = simulate_contact(P);
plotSoln(t,q0,q1,dq0,dq1,u, Fx,Fy,KE,PE);

[t,q0,q1,dq0,dq1,u,Fx,Fy,KE,PE] = simulate_minimal(P);
plotSoln(t,q0,q1,dq0,dq1,u, Fx,Fy,KE,PE);

subplot(2,4,4);
legend('dt','ode');