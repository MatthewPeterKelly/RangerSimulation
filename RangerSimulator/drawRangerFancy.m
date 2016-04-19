function drawRangerFancy(t,x,p)
% drawRangerFancy(t,x,p)
%
% Draws ranger

z = x(1:6);
dz = x(7:12);
u = x(13:15);
cam = x(16:17);   %Camera center coordinates
dist = x(18:19);  %Disturbance

clf;
subplot(1,3,[1,2]); hold on;  %For animation

colorFootOut = [0.8, 0.0, 0.2];
colorFootInn = [0.2, 0.0, 0.8];
colorLegOut = [0.7,0.0,0.3];
colorLegInn = [0.3,0.0,0.7];
colorGround = [0.4,0.1,0.1];

SizeJoint = 20;

DisturbanceScale = 0.2;

[p0,p1] = kinematics(z,[],[],p);

x = z(1);
y = z(2);
qf0 = z(3);
qf1 = z(4);
ql0 = z(5);
ql1 = z(6);

axisScale = 0.8*p.l;
extents = cam([1,1,2,2])' + axisScale*[-1,1,-1,1];

drawGround(extents([1,2]), colorGround, p.ground);

drawFoot(p1,qf1,p.r,p.d,colorFootInn,1);
drawLeg([x;y],ql1,p.l,colorLegInn);
plot(p1(1),p1(2),'k.','MarkerSize',SizeJoint);

drawFoot(p0,qf0,p.r,p.d,colorFootOut,0);
drawLeg([x;y],ql0,p.l,colorLegOut);
plot(p0(1),p0(2),'k.','MarkerSize',SizeJoint);

if max(abs(dist)) > 0.001
    dist = dist*DisturbanceScale;
    drawArrow([x;y]-dist,[x;y]-0.1*dist);  %Disturbance
end

plot(x,y,'k.','MarkerSize',SizeJoint);

title(sprintf('Ranger Simulation:  time = %6.4f',t));

axis equal; axis(extents); axis off;


%%%% Data:
subplot(1,3,3); hold on;   %For annotations:
textPosX = 0.1; 
textPosY = fliplr(linspace(0,1,19));
textFontSize = 14;
axis([0,1,0,1]); axis off;

text(textPosX, textPosY(2), sprintf('Hip X Pos:  %6.4f',z(1)), 'FontSize',textFontSize);
text(textPosX, textPosY(3), sprintf('Hip Y Pos:  %6.4f',z(2)), 'FontSize',textFontSize);
text(textPosX, textPosY(4), sprintf('Foot 0 Angle:  %6.4f',z(3)), 'FontSize',textFontSize);
text(textPosX, textPosY(5), sprintf('Foot 1 Angle:  %6.4f',z(4)), 'FontSize',textFontSize);
text(textPosX, textPosY(6), sprintf('Leg 0 Angle:  %6.4f',z(5)), 'FontSize',textFontSize);
text(textPosX, textPosY(7), sprintf('Leg 1 Angle:  %6.4f',z(6)), 'FontSize',textFontSize);

text(textPosX, textPosY(9), sprintf('Hip X Vel:  %6.4f',dz(1)), 'FontSize',textFontSize);
text(textPosX, textPosY(10), sprintf('Hip Y Vel:  %6.4f',dz(2)), 'FontSize',textFontSize);
text(textPosX, textPosY(11), sprintf('Foot 0 Rate:  %6.4f',dz(3)), 'FontSize',textFontSize);
text(textPosX, textPosY(12), sprintf('Foot 1 Rate:  %6.4f',dz(4)), 'FontSize',textFontSize);
text(textPosX, textPosY(13), sprintf('Leg 0 Rate:  %6.4f',dz(5)), 'FontSize',textFontSize);
text(textPosX, textPosY(14), sprintf('Leg 1 Rate:  %6.4f',dz(6)), 'FontSize',textFontSize);

text(textPosX, textPosY(16), sprintf('Ankle 0 Torque:  %6.4f',u(1)), 'FontSize',textFontSize);
text(textPosX, textPosY(18), sprintf('Ankle 1 Torque:  %6.4f',u(2)), 'FontSize',textFontSize);
text(textPosX, textPosY(17), sprintf('Hip Torque:  %6.4f',u(3)), 'FontSize',textFontSize);

end

function drawGround(xBnd, color, p_ground)

xGnd = linspace(xBnd(1), xBnd(2), 100);
yGnd = groundModel(xGnd,p_ground);

plot(xGnd, yGnd,'Color',color,'LineWidth',5);

% Tick marks:
period = 0.25;
xLow = period*ceil(xBnd(1)/period);
xUpp = period*floor(xBnd(2)/period);
x = xLow:period:xUpp;
y = groundModel(x,p_ground);
bigMark = mod(x,4*period)==0;
scale = -0.02*(xBnd(2) - xBnd(1));
for i=1:length(x)
    if bigMark(i)
        plot([1,1]*x(i), [0, 2*scale]+y(i), 'Color',color,'LineWidth',5);
    else
        plot([1,1]*x(i), [0, scale]+y(i), 'Color',color,'LineWidth',3);
    end
end

% Horizontal dashed line
plot(xGnd, 0*yGnd -0.03,'k-','LineWidth',1);

end

function drawFoot(foot,q,r,d,color,id)
% Draws a foot for visualization
%
% INPUTS:
%   pFoot = [2 x 1] position vector for ankle joint [x;y]
%   qFoot = scalar absolute angle of the foot
%   r = radius of the foot
%   d = distance from virtual center to the ankle joint
%

% Compute the foot points in a reference frame with the ankle joint at the
% origin and the orientation is with q = 0;

% Call the patch command for the detailed shape of the foot
drawRangerFoot(foot,q,color);

%%% Draw the virtual circle for the foot:
% R = [cos(q), -sin(q);
%     sin(q), cos(q)];
% vc = R*[0;d];
% x = vc(1) + foot(1);
% y = vc(2) + foot(2);
% plot(x,y,'ko','LineWidth',1,'MarkerSize',5);  %Center
% rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1]);

%%% Draw the ID text on the foot to know which is which:
posText = foot - 0.7*(r-d)*[cos(q); sin(q)]; 
text(posText(1),posText(2),num2str(id),...
    'FontSize',12,'FontWeight','bold');

%%% Plot a dot where the controller thinks the contact point is:
contactColor = [10, 200, 150]/255;
[xContact, yContact] = footContact_cart(q);
plot(foot(1) + xContact, foot(2) + yContact, '.','MarkerSize', 8,'Color',contactColor);

end

function drawLeg(hip,q,l,color)
% Draws a leg for visualization
%
% INPUTS:
%   pHip = [2 x 1] position vector for ankle joint [x;y]
%   qLeg = scalar absolute angle of the foot
%   l = length of the leg%

% Compute the leg points in a reference frame with the ankle joint at the
% origin and the orientation is with q = 0;

r = 0.015*l;
phi = linspace(0,pi,10);
curveUpp = [r*cos(phi); r*sin(phi)];
curveLow = [fliplr(curveUpp(1,:)); -l - curveUpp(2,:)];

R = [cos(q), -sin(q);
    sin(q), cos(q)];

points = R*[curveUpp, curveLow, curveUpp(:,1)];

patch(hip(1) + points(1,:), hip(2) + points(2,:), color);

end