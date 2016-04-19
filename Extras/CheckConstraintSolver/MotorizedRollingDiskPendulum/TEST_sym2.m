% MAIN - Contact Simulation - Sanity check on rolling solver:
%
%

clc; clear;

initializeProblem;

%%%% THE KEY LINE:
[t,q0,q1,dq0,dq1,u,current, power,Fx,Fy,KE,PE,p0,pStar,p1] = simulate_sym2(P);


%%%% Animation: 
data = [p0;pStar;p1];
Anim.plotFunc = @(t,x)( drawDiskPendulum(t,x,P) );
Anim.speed = 0.5;
Anim.figNum = 100;
Anim.verbose = false;
% animate(t,data,Anim);

%%%% Plots:
figure(101); clf;
plotSoln(t,q0,q1,dq0,dq1,u,current, power,Fx,Fy,KE,PE);


