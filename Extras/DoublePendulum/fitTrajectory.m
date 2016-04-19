function ctrl = fitTrajectory(soln,dyn)
% traj = fitTrajectory(soln)
%
% This function fits a spline to the solution, in terms of the hip angle,
% rate, and torque:
%

nFit = 25;  %How many data points to use for fitting the trajectory

nKnot = 3;  %How many knot points?

S = soln(end);
tSpan = S.grid.time([1,end]);
t = linspace(tSpan(1), tSpan(2), nFit);

z = S.interp.state(t);
u = S.interp.control(t);

q1 = z(1,:);
q2 = z(2,:);
dq1 = z(3,:);
dq2 = z(4,:);

q = q2-q1;
dq = dq2-dq1;

com = autoGen_getCoM(q1,q2,dq1,dq2,dyn.d,dyn.l);
x = com(1,:);

% Mapping from relative horizontal CoM position to phase [0,1]
xLow = min(x);
xUpp = max(x);
ctrl.phase.xLow = min(x);
ctrl.phase.xUpp = max(x);

p = (x-xLow)/(xUpp-xLow);
pKnot = linspace(0,1,nKnot);

ctrl.hip.pp.u = fitSpline(p,u,pKnot);
ctrl.hip.pp.q = fitSpline(p,q,pKnot);
ctrl.hip.pp.dq = fitSpline(p,dq,pKnot);

% checkFit(ctrl.hip.pp,p,q,dq,u);
end



function checkFit(PP,p,q,dq,u)
%
% Makes a quick plot to see if the function actuall fits well
%

uFit = ppval(PP.u, p);
qFit = ppval(PP.q, p);
dqFit = ppval(PP.dq, p);

figure(152); clf;

subplot(3,1,1); hold on;
plot(p,u,'ko')
plot(p,uFit,'b-')
xlabel('x');
ylabel('u');

subplot(3,1,2); hold on;
plot(p,q,'ko')
plot(p,qFit,'b-')
xlabel('x');
ylabel('q');

subplot(3,1,3); hold on;
plot(p,dq,'ko')
plot(p,dqFit,'b-')
xlabel('x');
ylabel('dq');

end



