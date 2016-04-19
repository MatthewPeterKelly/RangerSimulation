function plotFullState(output)
%
% This function plots the full state of the robot (x), given the results of
% a simulation.
%

%%%% Pretty:
colorOut = [0.9, 0.2, 0.2];
colorInn = [0.2, 0.2, 0.9];

colorX = [0.5, 0.6, 0.1];
colorY = [0.1, 0.6, 0.5];

%%%% UnPack: 
t = output.t;
z = output.x(1:6,:);
dz = output.x(7:12,:);

x = z(1,:);
y = z(2,:);
phi0 = z(3,:);
phi1 = z(4,:);
th0 = z(5,:);
th1 = z(6,:);

dx = dz(1,:);
dy = dz(2,:);
dphi0 = dz(3,:);
dphi1 = dz(4,:);
dth0 = dz(5,:);
dth1 = dz(6,:);

%%%% Plotting:

subplot(2,3,1); hold on;
plot(t,x,'Color',colorX);
plot(t,y,'Color',colorY);
xlabel('time');
title('hip position');
legend('x','y');

subplot(2,3,4); hold on;
plot(t,dx,'Color',colorX);
plot(t,dy,'Color',colorY);
xlabel('time');
title('hip velocity');
legend('dx','dy');



subplot(2,3,2); hold on;
plot(t,th0,'Color',colorOut);
plot(t,th1,'Color',colorInn);
xlabel('time');
title('leg angle');
legend('th0','th1');

subplot(2,3,5); hold on;
plot(t,dth0,'Color',colorOut);
plot(t,dth1,'Color',colorInn);
xlabel('time');
title('leg rate');
legend('dth0','dth1');



subplot(2,3,3); hold on;
plot(t,phi0,'Color',colorOut);
plot(t,phi1,'Color',colorInn);
xlabel('time');
title('foot angle');
legend('phi0','phi1');

subplot(2,3,6); hold on;
plot(t,dphi0,'Color',colorOut);
plot(t,dphi1,'Color',colorInn);
xlabel('time');
title('foot rate');
legend('dphi0','dphi1');


end