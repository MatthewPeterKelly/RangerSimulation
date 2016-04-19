function plotRobotMotors(output,model)
%
% This function plots the robots motors - stuff like current, cost of
% transport, power, etc.
%

%%%% Pretty:
colorOut = [0.9, 0.2, 0.2];
colorInn = [0.2, 0.2, 0.9];
colorHip = [0.2, 0.8, 0.2];
colorRobot = [0.8, 0.3, 0.8];
colorX = [0.3, 0.5, 0.7];
colorY = [0.7, 0.5, 0.3];    


thickLine = 2;
textFontSize = 16;

%%%% UnPack: 
t = output.t;

p0 = output.power(1,:);  %power to outer ankles
p1 = output.power(2,:);  %power to inner ankles
pHip = output.power(3,:);  %power to hips

i0 = output.current(1,:);  %current to outer ankles
i1 = output.current(2,:);  %current to inner ankles
iHip = output.current(3,:);  %current to hips

xDist = output.dist(1,:);   %Disturbance applied at hip (x)
yDist = output.dist(2,:);   %Disturbance applied at hip (y)

pOverhead = model.motor.overheadPower;  %Power to sensors + computers
pTotal = p0 + p1 + pHip + pOverhead;   %Total power use

%%%% Plotting:

subplot(2,3,1); hold on;
plot(t,pTotal,'Color',colorRobot,'LineWidth',thickLine);
xlabel('time');
title('total power use');
ylabel('power (Watts)');


subplot(2,3,2); hold on;
plot(t,p0,'Color',colorOut,'LineWidth',thickLine);
plot(t,p1,'Color',colorInn,'LineWidth',thickLine);
xlabel('time');
title('Ankle Power');
ylabel('power (Watts)');
legend('p0','p1');


subplot(2,3,4); hold on;
plot(t,xDist,'Color',colorX,'LineWidth',thickLine);
plot(t,yDist,'Color',colorY,'LineWidth',thickLine);
xlabel('time');
title('Hip Disturbance');
ylabel('force (N)');
legend('xDist','yDist');


subplot(2,3,5); hold on;
plot(t,i0,'Color',colorOut,'LineWidth',thickLine);
plot(t,i1,'Color',colorInn,'LineWidth',thickLine);
xlabel('time');
title('Ankle Current');
ylabel('current (amps)');
legend('I0','I1');


subplot(2,3,3); hold on;
plot(t,pHip,'Color',colorHip,'LineWidth',thickLine);
xlabel('time (s)');
title('Hip Power');
ylabel('power (Watts)');


subplot(2,3,6); hold on;
plot(t,iHip,'Color',colorHip,'LineWidth',thickLine);
xlabel('time (s)');
title('Hip Current');
ylabel('power (Watts)');


end