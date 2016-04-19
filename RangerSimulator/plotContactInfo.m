function plotContactInfo(output)
%
% This function plots contact information for the robot
%

%%%% Pretty:
colorOut = [0.9, 0.2, 0.2];
colorInn = [0.2, 0.2, 0.9];


%%%% UnPack: 
t = output.t;

f0x = output.f(1,:);
f0y = output.f(2,:);
f1x = output.f(3,:);
f1y = output.f(4,:);

c0x = output.c(1,:);
c0y = output.c(2,:);
c1x = output.c(3,:);
c1y = output.c(4,:);

k0 = output.k(1,:);
k1 = output.k(2,:);

fsm = output.fsmMode;


%%%% Plotting:

subplot(2,3,1); hold on;
plot(t,f0x,'Color',colorOut);
plot(t,f1x,'Color',colorInn);
xlabel('time');
title('horizontal contact force');
legend('0','1');

subplot(2,3,4); hold on;
plot(t,f0y,'Color',colorOut);
plot(t,f1y,'Color',colorInn);
xlabel('time');
title('vertical contact force');
legend('0','1');


subplot(2,3,2); hold on;
plot(t,c0x,'Color',colorOut);
plot(t,c1x,'Color',colorInn);
xlabel('time');
title('horizontal contact point');
legend('0','1');

subplot(2,3,5); hold on;
plot(t,c0y,'Color',colorOut);
plot(t,c1y,'Color',colorInn);
xlabel('time');
title('vertical contact point');
legend('0','1');


subplot(2,3,3); hold on;
plot(t,fsm,'k-','LineWidth',4);
xlabel('time');
axis([t([1,end]),0,6]);
title('fsm mode');
text(0.04*t(end),4, {'0 = FL'; '1 = Sw0'; '2 = Pu0'; '3 = Ds0'; '4 = Sw1'; '5 = Pu1'; '6 = Ds1'});

subplot(2,3,6); hold on;
plot(t,k0+2*k1-1,'k-','LineWidth',4);
xlabel('time');
title('contact mode');
axis([t([1,end]),-1,2]);
text(0.1*t(end),-0.5,'0 = S0, 1 = S1, 2 = DS, -1 = FL');


end