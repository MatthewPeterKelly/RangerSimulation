function plotSoln(t,q0,q1,dq0,dq1,u,Fx,Fy,KE,PE)

subplot(2,4,1); hold on;
plot(t,q0)
xlabel('t')
ylabel('q0')

subplot(2,4,2); hold on;
plot(t,q1)
xlabel('t')
ylabel('q1')

subplot(2,4,5); hold on;
plot(t,dq0)
xlabel('t')
ylabel('dq0')

subplot(2,4,6); hold on;
plot(t,dq1)
xlabel('t')
ylabel('dq1')

subplot(2,4,3); hold on;
plot(t,Fx)
xlabel('t')
ylabel('Fx')

subplot(2,4,7); hold on;
plot(t,Fy)
xlabel('t')
ylabel('Fy')

subplot(2,4,4); hold on;
plot(t,KE+PE)
xlabel('t')
ylabel('E')

subplot(2,4,8); hold on;
plot(t,u)
xlabel('t')
ylabel('u')

end