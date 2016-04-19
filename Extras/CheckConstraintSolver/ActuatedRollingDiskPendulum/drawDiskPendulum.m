function drawDiskPendulum(t,data,param)

x0 = data(1,:);
y0 = data(2,:);
xStar = data(3,:);
yStar = data(4,:);
x1 = data(5,:);
y1 = data(6,:);

r = param.r;  % Disk radius
a = param.a;  % pendulum joint offset
b = param.b;  % pendulum length (joint to CoM)

scale = r+a+b;
cameraPos = [0;0];  %[x0;y0];
xBnd = cameraPos(1) + 1.5*scale*[-1,1];
yBnd = cameraPos(2) + scale*[-1.2, 1.4];
extents = [xBnd,yBnd];

% Figure stuff:
clf; hold on;

% Draw Ground:
colorGround = [102, 53, 44]/255;
drawGround(xBnd, colorGround, param.slope);


% Draw Disk:
colorDisk = [47, 47, 212]/255;
rectangle('Position',[x0-r,y0-r,2*r,2*r],'Curvature',[1,1],...
    'LineWidth',3,'EdgeColor',colorDisk);
plot([x0,xStar],[y0, yStar],'LineWidth',3,'Color',colorDisk);
plot(x0,y0,'k.','MarkerSize',25);


% Draw Pendulum:
colorPendulum = [171, 34, 58]/255;
plot([xStar;x1],[yStar;y1],'LineWidth',4,'Color',colorPendulum)
plot(xStar, yStar,'k.','MarkerSize',25);

% Title:
title(sprintf('Disk Pendulum,  t = %4.4f',t));

% Fix axis scaling:
axis(extents); axis equal;

end




function drawGround(xBnd, color, slope)

xGnd = linspace(xBnd(1), xBnd(2), 2);
yGnd = xGnd*slope;

plot(xGnd, yGnd,'Color',color,'LineWidth',5);

% Tick marks:
period = 0.25;
xLow = period*ceil(xBnd(1)/period);
xUpp = period*floor(xBnd(2)/period);
x = xLow:period:xUpp;
y = x*slope;
bigMark = mod(x,4*period)==0;
scale = -0.02*(xBnd(2) - xBnd(1));
for i=1:length(x)
    if bigMark(i)
        plot([1,1]*x(i), [0, 2*scale]+y(i), 'Color',color,'LineWidth',5);
    else
        plot([1,1]*x(i), [0, scale]+y(i), 'Color',color,'LineWidth',3);
    end
end

end

