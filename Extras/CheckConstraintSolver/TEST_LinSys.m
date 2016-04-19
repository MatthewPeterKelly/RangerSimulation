% Test -- Linear System -- constraint sovler
%
% This script does a bit of linear systems math to play around with the
% parameters of the constraint solver.
%
% This code is used to test a (highly-simplified) version of the constraint
% simulator that I used in the Ranger simulator.
%
% In this case z(1,:) represents the constraint violation and dz(2,:)
% represents the rate of change in constraint violation. This code would
% only be running while the constraint is in violation (ie. right after the
% foot strikes the ground).
%

clc; clear;

z0 = [1;1];   %Velocity

h = 1;  %Time step

A = [0,1;1,h];
B = [h;h*h];
P = [0.1,0.0];  %Eigen values
K = place(A,B,P);
% NOTE: h*K = constant for given A,B,P


n = 10;
z = zeros(2,n);
z(:,1) = z0;

for i=2:n
    u = -K*z(:,i-1);
    z(:,i) = A*z(:,i-1) + B*u;
end
x = z(1,:);
v = z(2,:);

figure(1); clf;

subplot(2,1,1);
plot(x,'ko')
xlabel('step')
ylabel('x')
% set(gca,'yScale','log')

subplot(2,1,2);
plot(v,'ko')
xlabel('step')
ylabel('v')
% set(gca,'yScale','log')


