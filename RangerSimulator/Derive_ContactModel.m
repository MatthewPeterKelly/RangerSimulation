% Derive Contact model for the foot
%
% For each angle of the foot, compute the place where it is expected to
% contact the ground.
%
%

figure(1); clf; hold on;

nData = 151;

qFoot = linspace(-pi,pi,nData);   %MUST BE ODD!

color = [0.1, 0.6, 0.2];
origin = [0;0];

%%% for debugging:
for i=1:length(qFoot)
    clf;
    
    drawRangerFoot(origin,qFoot(i),color); hold on;
    
    getHeight = @(q)( getFootPoint(qFoot(i),q));
    
    qSoln = fminsearch(getHeight,0);
    
    [y,x] = getFootPoint(qFoot(i),qSoln);
    plot(x,y,'ko');
    pause(0.02);
    
end

x = zeros(size(qFoot));
y = zeros(size(qFoot));
r = zeros(size(qFoot));
q = zeros(size(qFoot));
for i=1:length(qFoot)

    getHeight = @(qTest)( getFootPoint(qFoot(i),qTest));
    
    q(i) = fminsearch(getHeight,0);    
    [y(i),x(i),r(i)] = getFootPoint(qFoot(i),q(i));
    
end

figure(2); clf;

subplot(2,2,1);
plot(qFoot,x)
xlabel('foot angle')
ylabel('horizontal contact')

subplot(2,2,3);
plot(qFoot,y)
xlabel('foot angle')
ylabel('vertical contact')

subplot(2,2,2);
plot(qFoot,r)
xlabel('foot angle')
ylabel('contact radius')

subplot(2,2,4);
plot(qFoot,q)
xlabel('foot angle')
ylabel('contact angle')


%%%% Print out the data-set for hard-coding:

fprintf('qFoot = [');
for i=1:nData
    if i<nData
    fprintf('%8.8f, ',qFoot(i));
    else
       fprintf('%8.8f];  %% Foot orientation \n',qFoot(i)); 
    end
end

fprintf('x = [');
for i=1:nData
    if i<nData
    fprintf('%8.8f, ',x(i));
    else
       fprintf('%8.8f];  %% relative horizontal position \n',x(i)); 
    end
end

fprintf('y = [');
for i=1:nData
    if i<nData
    fprintf('%8.8f, ',y(i));
    else
       fprintf('%8.8f];  %% relative verical position \n',y(i)); 
    end
end

fprintf('r = [');
for i=1:nData
    if i<nData
    fprintf('%8.8f, ',r(i));
    else
       fprintf('%8.8f];  %% distance \n',r(i)); 
    end
end

fprintf('q = [');
for i=1:nData
    if i<nData
    fprintf('%8.8f, ',q(i));
    else
       fprintf('%8.8f];  %% relative angle \n',q(i)); 
    end
end


