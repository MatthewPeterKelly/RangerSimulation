% TEST - minimal coordinate version of disk-pendulum system
%
%

clc; clear;

initializeProblem;

[t,q0,q1,dq0,dq1,u,Fx,Fy,KE,PE,p0,pStar,p1] = simulate_minimal(P);


%%%% Animation: 
data = [p0;pStar;p1];
Anim.plotFunc = @(t,x)( drawDiskPendulum(t,x,P) );
Anim.speed = 0.5;
Anim.figNum = 100;
Anim.verbose = false;
% animate(t,data,Anim);


%%%% Plots:
figure(102); clf;
plotSoln(t,q0,q1,dq0,dq1,u, Fx,Fy,KE,PE);


