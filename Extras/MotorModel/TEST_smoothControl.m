% TEST_smoothControl.m
%
% Tests the smooth controller


kp = 25;
kd = 10;

uMax = 5;

xRef = 0.0;
vRef = 0.0;

x0 = 1.0;
v0 = 0.0;

dt = 0.005;
tSpan = [0, 2];
nGrid = ceil(diff(tSpan)/dt);
t = zeros(1,nGrid);
x = zeros(1,nGrid);
v = zeros(1,nGrid);
u = zeros(1,nGrid);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                TEST ONE:  simple non-linear controller                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%

x(1) = x0;
v(1) = v0;
P = makeStruct(kp,kd,uMax,xRef,vRef);

for i=1:nGrid
   u(i) = smoothControl(x(i),v(i),P); 
   if i<nGrid
      t(i+1) = t(i) + dt;
      v(i+1) = v(i) + dt*u(i);
      x(i+1) = x(i) + dt*v(i+1);  %Symplectic euler 
   end
end

figure(1); clf;
title('smooth non-linear controller')

subplot(3,1,1);
plot(t,x)
xlabel('t');
ylabel('x');

subplot(3,1,2);
plot(t,v)
xlabel('t');
ylabel('v');

subplot(3,1,3);
plot(t,u)
xlabel('t');
ylabel('u');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                TEST TWO:  successive linearization                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%

nSubStep = 4;

x(1) = x0;
v(1) = v0;
P = makeStruct(kp,kd,uMax,xRef,vRef);

for i=1:nGrid
   if mod(i-1,nSubStep)==0
       [~, KK] = smoothControl(x(i),v(i),P); 
   end  
   u(i) = KK.u0 - KK.kp*x(i) - KK.kd*v(i); 
   if i<nGrid
      t(i+1) = t(i) + dt;
      v(i+1) = v(i) + dt*u(i);
      x(i+1) = x(i) + dt*v(i+1);  %Symplectic euler 
   end
end

figure(2); clf;
title('Successive linearization')

subplot(3,1,1);
plot(t,x)
xlabel('t');
ylabel('x');

subplot(3,1,2);
plot(t,v)
xlabel('t');
ylabel('v');

subplot(3,1,3);
plot(t,u)
xlabel('t');
ylabel('u');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                TEST THREE:  gradient check                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%


% Check a few test points
xTest = 0.4;
vConst = -0.2;

x = linspace(-1,1,100);
v = vConst*ones(size(x));
u = smoothControl(x,v,P);

[~, KK] = smoothControl(xTest,vConst,P);
xx = linspace(xTest-0.2, xTest+0.2, 25);
uu = KK.u0 - KK.kp*xx - KK.kd*vConst;

figure(3); clf; hold on;
title('Linearization check');

plot(x,u)
plot(xx,uu)
plot(xTest,smoothControl(xTest,vConst,P),'ko')
legend('exact','linearized')


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                TEST TWO:  successive linearization, non-zero ref        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%

nSubStep = 4;

xRef = -0.3;

x(1) = x0;
v(1) = v0;
P = makeStruct(kp,kd,uMax,xRef,vRef);

for i=1:nGrid
   if mod(i-1,nSubStep)==0
       [~, KK] = smoothControl(x(i),v(i),P); 
   end  
   u(i) = KK.u0 - KK.kp*x(i) - KK.kd*v(i); 
   if i<nGrid
      t(i+1) = t(i) + dt;
      v(i+1) = v(i) + dt*u(i);
      x(i+1) = x(i) + dt*v(i+1);  %Symplectic euler 
   end
end

figure(4); clf;
title('Successive linearization')

subplot(3,1,1);
plot(t,x)
xlabel('t');
ylabel('x');

subplot(3,1,2);
plot(t,v)
xlabel('t');
ylabel('v');

subplot(3,1,3);
plot(t,u)
xlabel('t');
ylabel('u');