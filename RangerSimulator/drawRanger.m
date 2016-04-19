function drawRanger(z,p)
% drawRanger(z,p)
%
% Draws ranger


clf; hold on;

colorFootOne = [0.8, 0.0, 0.2];
colorFootTwo = [0.2, 0.0, 0.8];
colorLegOne = [0.7,0.0,0.3];
colorLegTwo = [0.3,0.0,0.7];

colorGround = [0.4,0.1,0.1];

SizeJoint = 25;

[p1,p2] = kinematics(z,[],[],p);

x = z(1);
y = z(2);
qf1 = z(3);
qf2 = z(4);
ql1 = z(5);
ql2 = z(6);

% [p1v,p2v] = kinematicsVc(z,[],p);
% [p1c, m1] = getContactPoint(p1v,qf1,p);   %c = contact point, m = isRolling
% [p2c, m2] = getContactPoint(p2v,qf2,p);

xGround = linspace(-1,1,100);
yGround = groundModel(xGround,p.ground);
plot(xGround,yGround,'color',colorGround);

drawFoot(p1,qf1,p.r,p.d,colorFootOne,1);
drawLeg([x;y],ql1,p.l,colorLegOne);

drawFoot(p2,qf2,p.r,p.d,colorFootTwo,2);
drawLeg([x;y],ql2,p.l,colorLegTwo);

plot(p1(1),p1(2),'k.','MarkerSize',SizeJoint);
plot(x,y,'k.','MarkerSize',SizeJoint);
plot(p2(1),p2(2),'k.','MarkerSize',SizeJoint);

% Plot the contact points
% if m1, s1 = 'kx'; else s1 = 'ko'; end;
% if m2, s2 = 'kx'; else s2 = 'ko'; end;
% plot(p1c(1),p1c(2),s1,'LineWidth',3,'MarkerSize',6)
% plot(p2c(1),p2c(2),s2,'LineWidth',3,'MarkerSize',6)

axis equal; 
pause(0.001);

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
R = [cos(q), -sin(q);
    sin(q), cos(q)];
vc = R*[0;d];
x = vc(1) + foot(1);
y = vc(2) + foot(2);
plot(x,y,'kx','LineWidth',3,'MarkerSize',6);  %Center
rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1]);

% Draw the ID text on the foot to know which is which:
posText = foot - 0.7*(r-d)*[cos(q); sin(q)]; 
text(posText(1),posText(2),num2str(id),...
    'FontSize',12,'FontWeight','bold');

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