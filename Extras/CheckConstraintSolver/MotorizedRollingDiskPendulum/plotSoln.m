function plotSoln(t,q0,q1,dq0,dq1,u,current, power, Fx,Fy,KE,PE,marker)

if nargin < 13
    marker = 'k-';
end

subplot(3,3,1); hold on;
plot(t,q0,marker)
xlabel('t')
ylabel('q0')

subplot(3,3,2); hold on;
plot(t,q1,marker)
xlabel('t')
ylabel('q1')

subplot(3,3,4); hold on;
plot(t,dq0,marker)
xlabel('t')
ylabel('dq0')

subplot(3,3,5); hold on;
plot(t,dq1,marker)
xlabel('t')
ylabel('dq1')

subplot(3,3,7); hold on;
plot(t,Fx,marker)
xlabel('t')
ylabel('Fx')

subplot(3,3,8); hold on;
plot(t,Fy,marker)
xlabel('t')
ylabel('Fy')

subplot(3,3,3); hold on;
plot(t,u,marker)
xlabel('t')
ylabel('u')

subplot(3,3,6); hold on;
plot(t,current,marker)
xlabel('t')
ylabel('current')

subplot(3,3,9); hold on;
plot(t,power,marker)
xlabel('t')
ylabel('power')

end