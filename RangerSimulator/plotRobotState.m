function plotRobotState(output,model)
%
% This function plots the robot state - relative joint angles, rates, and
% torques.
%

%%%% Pretty:
colorOut = [0.9, 0.2, 0.2];
colorInn = [0.2, 0.2, 0.9];
colorHip = [0.8, 0.3, 0.8];
colorRef = [0.5, 0.1, 0.5];

thickLine = 2;
thinLine = 1;

%%%% UnPack: 
t = output.t;
angAbs = output.x(3:6,:);
rateAbs = output.x(9:12,:);
[angRel, rateRel] = getAngRel(angAbs,rateAbs,model.dyn.Phi);

qr = angRel(1,:);
qh = angRel(2,:);
q0 = angRel(3,:);
q1 = angRel(4,:);

dqr = rateRel(1,:);
dqh = rateRel(2,:);
dq0 = rateRel(3,:);
dq1 = rateRel(4,:);

u0 = output.u(1,:);
u1 = output.u(2,:);
uHip = output.u(3,:);

u0_ref = output.uIdeal(1,:);
u1_ref = output.uIdeal(2,:);
uHip_ref = output.uIdeal(3,:);

q0_ref = output.qRef(1,:);
q1_ref = output.qRef(2,:);
qh_ref = output.qRef(3,:);

dq0_ref = output.dqRef(1,:);
dq1_ref = output.dqRef(2,:);
dqh_ref = output.dqRef(3,:);

%%%% Plotting:

subplot(2,3,1); hold on;
plot(t,qr,'Color',colorRef,'LineWidth',thickLine);
plot(t,qh,'Color',colorHip,'LineWidth',thickLine);
plot(t,qh_ref,'Color',colorHip,'LineWidth',thinLine);
xlabel('time');
title('hip angles');
legend('qr','qh');

subplot(2,3,4); hold on;
plot(t,dqr,'Color',colorRef,'LineWidth',thickLine);
plot(t,dqh,'Color',colorHip,'LineWidth',thickLine);
plot(t,dqh_ref,'Color',colorHip,'LineWidth',thinLine);
xlabel('time');
title('hip rates');
legend('dqr','dqh');


subplot(2,3,2); hold on;
plot(t,q0,'Color',colorOut,'LineWidth',thickLine);
plot(t,q1,'Color',colorInn,'LineWidth',thickLine);
plot(t,q0_ref,'Color',colorOut,'LineWidth',thinLine);
plot(t,q1_ref,'Color',colorInn,'LineWidth',thinLine);
xlabel('time');
title('ankle angles');
legend('q0','q1');

subplot(2,3,5); hold on;
plot(t,dq0,'Color',colorOut,'LineWidth',thickLine);
plot(t,dq1,'Color',colorInn,'LineWidth',thickLine);
plot(t,dq0_ref,'Color',colorOut,'LineWidth',thinLine);
plot(t,dq1_ref,'Color',colorInn,'LineWidth',thinLine);
xlabel('time');
title('ankle rates');
legend('dq0','dq1');



subplot(2,3,3); hold on;
plot(t,u0,'Color',colorOut,'LineWidth',thickLine);
plot(t,u1,'Color',colorInn,'LineWidth',thickLine);
plot(t,u0_ref,'Color',colorOut,'LineWidth',thinLine);
plot(t,u1_ref,'Color',colorInn,'LineWidth',thinLine);
xlabel('time');
title('ankle torques');
legend('u0','u1');

subplot(2,3,6); hold on;
plot(t,uHip,'Color',colorHip,'LineWidth',thickLine);
plot(t,uHip_ref,'Color',colorHip,'LineWidth',thinLine);
xlabel('time');
title('hip torque');
legend('uHip');

end