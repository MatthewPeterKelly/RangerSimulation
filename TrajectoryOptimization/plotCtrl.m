function plotCtrl(traj)

% This function plots the trajectory in phase space

figure(151); clf;

dotColor = [0.3,0.5,0.9];
lineWidth = 1;

% Store the original trajectory in phase space:
M.p = traj.ref.c*traj.phase.z;
M.dp = traj.ref.c*traj.phase.dz;
M.ddp = traj.ref.c*traj.phase.ddz;
M.h = traj.ref.H*traj.phase.z;
dhdt = traj.ref.H*traj.phase.dz;
ddhddt = traj.ref.H*traj.phase.ddz;
M.dh = (dhdt)./(ones(3,1)*M.dp);  %Chain rule
M.ddh = (ddhddt - M.dh.*(ones(3,1)*M.ddp))./((ones(3,1)*M.dp).^2);  %Chain rule
M.u = traj.phase.u;

% Interpolate the reference trajectory and evaluate nominal controller:
P.p = M.p;
P.h = ppval(traj.ref.pp.h,P.p);
P.dh = ppval(traj.ref.pp.dh,P.p);
P.ddh = ppval(traj.ref.pp.ddh,P.p);

nTime = length(P.p);
P.u = zeros(3,nTime);
for i=1:nTime
[~, P.u(:,i)] = systemDynamics(...
    traj.phase.z(:,i), traj.phase.dz(:,i),...
    traj.config.dyn, traj.ref);
end

%%%% Plot both the original and the controller reference trajectories:

subplot(4,3,1); hold on;
plot(M.p,M.h(1,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.h(1,:),'k-','LineWidth',lineWidth);
title('q1 - stance foot')

subplot(4,3,2); hold on;
plot(M.p,M.h(2,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.h(2,:),'k-','LineWidth',lineWidth);
title('q2 - swing leg')

subplot(4,3,3); hold on;
plot(M.p,M.h(3,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.h(3,:),'k-','LineWidth',lineWidth);
title('q3 - swing foot')


subplot(4,3,4); hold on;
plot(M.p,M.dh(1,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.dh(1,:),'k-','LineWidth',lineWidth);
title('dq1 - stance foot')

subplot(4,3,5); hold on;
plot(M.p,M.dh(2,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.dh(2,:),'k-','LineWidth',lineWidth);
title('dq2 - swing leg')

subplot(4,3,6); hold on;
plot(M.p,M.dh(3,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.dh(3,:),'k-','LineWidth',lineWidth);
title('dq3 - swing foot')


subplot(4,3,7); hold on;
plot(M.p,M.ddh(1,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.ddh(1,:),'k-','LineWidth',lineWidth);
title('ddq1 - stance foot')

subplot(4,3,8); hold on;
plot(M.p,M.ddh(2,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.ddh(2,:),'k-','LineWidth',lineWidth);
title('ddq2 - swing leg')

subplot(4,3,9); hold on;
plot(M.p,M.ddh(3,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.ddh(3,:),'k-','LineWidth',lineWidth);
title('ddq3 - swing foot')

subplot(4,3,10); hold on;
plot(M.p,M.u(1,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.u(1,:),'k-','LineWidth',lineWidth);
title('u1 - stance foot')
xlabel('p - swing leg')

subplot(4,3,11); hold on;
plot(M.p,M.u(2,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.u(2,:),'k-','LineWidth',lineWidth);
title('uHip - swing leg')
xlabel('p - swing leg')

subplot(4,3,12); hold on;
plot(M.p,M.u(3,:),'.','Color',dotColor,'MarkerSize',10);
plot(M.p,P.u(3,:),'k-','LineWidth',lineWidth);
title('u2 - swing foot')
xlabel('p - swing leg')

end