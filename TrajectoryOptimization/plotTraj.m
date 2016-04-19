function plotTraj(traj)

colorOne = [0.8,0.0,0.2];
colorTwo = [0.2,0.0,0.8];
colorHip = [0.5,0.0,0.5];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Unpack data                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

t = traj.phase.t;

qf1 = traj.phase.z(1,:);
qf2 = traj.phase.z(2,:);
ql1 = traj.phase.z(3,:);
ql2 = traj.phase.z(4,:);

u1 = traj.phase.u(1,:);
uHip = traj.phase.u(2,:);
u2 = traj.phase.u(3,:);

f1x = traj.phase.f(1,:);
f1y = traj.phase.f(2,:);
f2x = traj.phase.f(3,:);
f2y = traj.phase.f(4,:);

[f1q, f1r] = cart2pol(f1x,f1y); f1q = f1q - pi/2;
[f2q, f2r] = cart2pol(f2x,f2y); f2q = f2q - pi/2;

pHip = traj.phase.pHip;
p1 = traj.phase.p1;
p2 = traj.phase.p2;

dpHip = traj.phase.dpHip;
dp1 = traj.phase.dp1;
dp2 = traj.phase.dp2;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Cartesian Plots                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


figure(143); clf;

nSubRow = 2;
nSubCol = 3;
idxSub = 0;   %Holds the current subplot index

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,p1(1,:),'color',colorOne);
plot(t,p2(1,:),'color',colorTwo);
plot(t,pHip(1,:),'color',colorHip);
title('x - position ')
xlabel('time')
ylabel('distance (m)')

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,p1(2,:),'color',colorOne);
plot(t,p2(2,:),'color',colorTwo);
plot(t,pHip(2,:),'color',colorHip);
title('y - position ')
xlabel('time')
ylabel('distance (m)')

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,f1x,'color',colorOne);
plot(t,f2x,'color',colorTwo);
title('x - contact force')
xlabel('time')
ylabel('force (N)')

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,dp1(1,:),'color',colorOne);
plot(t,dp2(1,:),'color',colorTwo);
plot(t,dpHip(1,:),'color',colorHip);
title('x - speed ')
xlabel('time')
ylabel('speed (m/s)')

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,dp1(2,:),'color',colorOne);
plot(t,dp2(2,:),'color',colorTwo);
plot(t,dpHip(2,:),'color',colorHip);
title('y - speed ')
xlabel('time')
ylabel('distance (m/s)')

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,f1y,'color',colorOne);
plot(t,f2y,'color',colorTwo);
title('y - contact force')
xlabel('time')
ylabel('force (N)')

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Plots related to angles                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

figure(142); clf;

nSubRow = 2;
nSubCol = 3;
idxSub = 0;   %Holds the current subplot index

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,180*qf1/pi,'color',colorOne);
plot(t,180*qf2/pi,'color',colorTwo);
title('foot angles')
xlabel('time')
ylabel('angle from vertical (deg)')

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,u1,'color',colorOne);
plot(t,u2,'color',colorTwo);
title('ankle torques')
xlabel('time')
ylabel('torque (Nm)')

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,f1r,'color',colorOne);
plot(t,f2r,'color',colorTwo);
xlabel('time')
ylabel('force (N)');
title('contact force magnitude')

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,180*ql1/pi,'color',colorOne);
plot(t,180*ql2/pi,'color',colorTwo);
title('leg angles')
xlabel('time')
ylabel('angle from vertical (deg)')

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,uHip,'color',colorHip);
title('hip torque')
xlabel('time')
ylabel('torque (Nm)')

idxSub = idxSub+1; subplot(nSubRow,nSubCol,idxSub); hold on; 
plot(t,180*f1q/pi,'color',colorOne);
plot(t,180*f2q/pi,'color',colorTwo);
title('contact force angle')
xlabel('time')
ylabel('angle from vertical (deg)')

end