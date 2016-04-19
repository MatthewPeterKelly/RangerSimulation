% Derive_footModel.m
%
% 

% Basic foot parameters
r = 200;  % Radius of the foot
d = 140;  % Eccentricity of the ankle joint from virtual center
phiHeel = -5*pi/180;
phiToe = 30*pi/180;
c = r-d;  %General scale factor (min distance from sole to axle)

% Plotting stuff:
figure(142); clf; hold on;
subplot(1,2,1); hold on; title('Cartesian'); 
xlabel('x'); ylabel('y');  axis equal
subplot(1,2,2); hold on; title('Polar');
xlabel('th'); ylabel('r'); 

colorSole = [0.9, 0.1, 0.0];
colorBack = [0.0, 0.1, 0.9];
colorTop = [0.1, 0.7,0.1];
colorFoot = [0.1,0.1,0.1];
markerSize = 30;
lineWidth = 4;

% Plot the virtual center and axle joint:
subplot(1,2,1);  %Cartesian
plot(0,0,'ko','LineWidth',3,'MarkerSize',10);
plot(0,d,'k+','LineWidth',2,'MarkerSize',5);

%%%% Construct the curve for the sole of the foot:
phi = linspace(phiHeel,phiToe,25);
sole.x = r*sin(phi);
sole.y = d-r*cos(phi);
[sole.th, sole.r] = cart2pol(-sole.y,sole.x);

sole.Th = [sole.th(1),sole.th(end)];
sole.P = fitPoly5(sole.Th,sole.th,sole.r);
sole.fit.th = sole.th;
sole.fit.r = pwPoly5(sole.Th,sole.P, sole.fit.th);
[xTmp,yTmp] = pol2cart(sole.fit.th, sole.fit.r);
sole.fit.x = yTmp; sole.fit.y = -xTmp;

subplot(1,2,1);  %Cartesian
plot(sole.x,sole.y,'.','MarkerSize',markerSize,'color',colorSole);
plot(sole.fit.x, sole.fit.y,'color',colorSole,'LineWidth',lineWidth);
subplot(1,2,2);  %Polar
plot(sole.th,sole.r,'.','MarkerSize',markerSize,'color',colorSole);
plot(sole.fit.th, sole.fit.r,'color',colorSole,'LineWidth',lineWidth);


%%%% Construct the curve for the back of the foot:
back.n = 10;
back.offset = 0.15*c;
back.x = (sole.x(1) - back.offset)*ones(1,back.n);
back.y = linspace(0,-0.7*c, back.n);

[back.th, back.r] = cart2pol(-back.y,back.x);

back.Th = [back.th(1),back.th(end)];
back.P = fitPoly5(back.Th,back.th,back.r);
back.fit.th = back.th;
back.fit.r = pwPoly5(back.Th,back.P, back.fit.th);
[xTmp,yTmp] = pol2cart(back.fit.th, back.fit.r);
back.fit.x = yTmp; back.fit.y = -xTmp;

subplot(1,2,1);  %Cartesian
plot(back.x,back.y,'.','MarkerSize',markerSize,'color',colorBack);
plot(back.fit.x, back.fit.y,'color',colorBack,'LineWidth',lineWidth);
subplot(1,2,2);  %Polar
plot(back.th,back.r,'.','MarkerSize',markerSize,'color',colorBack);
plot(back.fit.th, back.fit.r,'color',colorBack,'LineWidth',lineWidth);


%%%% Construct the curve for the top of the foot:
top.n = 40;
top.x0 = sole.x(end) - 0.2*c;
top.y0 = sole.y(end) + 0.4*c;
top.x1 = 0.6*c;
top.y1 = 0.3*c;
top.x = linspace(top.x0,top.x1,top.n);
top.y = linspace(top.y0,top.y1,top.n);


[top.th, top.r] = cart2pol(-top.y,top.x);

top.Th = [top.th(1),top.th(end)];
top.P = fitPoly5(top.Th,top.th,top.r);
top.fit.th = top.th;
top.fit.r = pwPoly5(top.Th,top.P, top.fit.th);
[xTmp,yTmp] = pol2cart(top.fit.th, top.fit.r);
top.fit.x = yTmp; top.fit.y = -xTmp;

subplot(1,2,1);  %Cartesian
plot(top.x,top.y,'.','MarkerSize',markerSize,'color',colorTop);
plot(top.fit.x, top.fit.y,'color',colorTop,'LineWidth',lineWidth);
subplot(1,2,2);  %Polar
plot(top.th,top.r,'.','MarkerSize',markerSize,'color',colorTop);
plot(top.fit.th, top.fit.r,'color',colorTop,'LineWidth',lineWidth);



%%%% Construct the curve for the entire foot:
foot.n = 400;
foot.Th = [back.Th, sole.Th, top.Th, back.Th(1)+2*pi];
foot.P = [back.P, sole.P, top.P, back.P(:,1)];
foot.th = linspace(foot.Th(1), foot.Th(end),foot.n);
foot.r = pwPoly5(foot.Th,foot.P,foot.th);
[xTmp,yTmp] = pol2cart(foot.th, foot.r);
foot.x = yTmp; foot.y = -xTmp;

subplot(1,2,1);  %Cartesian
plot(foot.x, foot.y,'color',colorFoot,'LineWidth',2);
subplot(1,2,2);  %Polar
plot(foot.th, foot.r,'color',colorFoot,'LineWidth',2);

%%%% Compute the best points to use for drawing the foot:
nGrid = [2,5,6,6,2,7];  %How many points per segment
nSegment = length(nGrid);
thPatch = zeros(1, sum(nGrid)-length(nGrid) + 1);
idxLow = 1;
for i=1:nSegment
    idxUpp = idxLow + nGrid(i)-2;
    thTmp = linspace(foot.Th(i), foot.Th(i+1), nGrid(i));
    thPatch(idxLow:idxUpp) = thTmp(1:(end-1));
    idxLow = idxUpp + 1;
end
thPatch(end) = foot.Th(end);
rPatch = pwPoly5(foot.Th,foot.P,thPatch);
[yPatch, xPatch] = pol2cart(thPatch,rPatch); yPatch = -yPatch;


%%%% Save the foot paramters:
P = foot.P/1000; r = r/1000; d = d/1000; %Convert to meters (not mm)
xPatch = xPatch/1000; yPatch = yPatch/1000;
Th = foot.Th;
paramVec = [Th(:); reshape(P,numel(foot.P),1)];
save('footParameters.mat',...
    'P','Th','paramVec',...
    'xPatch','yPatch',...
    'phiHeel','phiToe','r','d');
r = r*1000; d = d*1000; %Convert back to mm



