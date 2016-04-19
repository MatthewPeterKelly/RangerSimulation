function drawRanger(X, p)
%drawRanger(X,config)
%
% This function draws Ranger, given the configuration state.
%
% INPUTS:
%   X = [10 x 1] pose vector
%       X(1)  = qf1  = foot one absolute angle
%       X(2)  = qf2  = foot two absolute angle
%       X(3)  = ql1  = leg one absolute angle
%       X(4)  = ql2  = leg two absolute angle
%       X(5)  = pHip_x = hip x-coordinate
%       X(6)  = pHip_y = hip y-coordinate
%       X(7)  = p1_x  = ankle joint one, x-coordinate
%       X(8)  = p1_y  = ankle joint one, y-coordinate
%       X(9)  = p2_x  = ankle joint two, x-coordinate
%       X(10) = p2_y  = ankle joint two, y-coordinate
%   p = struct of physical parameters for Ranger
%
% NOTES:
%   States:
%       1 = hip horizontal position
%       2 = hip vertical position
%       3 = stance foot one angle
%       4 = swing foot two angle
%       5 = stance leg one angle
%       6 = swing leg two angle
%
%

% Plotting Parameters
colorFootOne = [0.8,0.0,0.2];
colorFootTwo = [0.2,0.0,0.8];
colorLegOne = [0.7,0.0,0.3];
colorLegTwo = [0.3,0.0,0.7];

colorGround = [0.6,0.2,0.1];

SizeJoint = 25;

% Unpack variables:
phiLow = p.angleHeel;
phiUpp = p.angleToe;
phiBnd = [phiLow, phiUpp];

qf1 = X(1);
qf2 = X(2);
ql1 = X(3);
ql2 = X(4);
pHip = X(5:6);
p1 = X(7:8);
p2 = X(9:10);

x = pHip(1);
y = pHip(2);

% Compute a good frame for the window:
legLenMax = p.l + p.r;
xLow = x - legLenMax;
xUpp = x + legLenMax;
yUpp = legLenMax;
yLow = -p.d;
axis([xLow,xUpp,yLow,yUpp]);

%%%% Begin plotting:

clf; hold on; 

plot([-1,1],[0,0],'color',colorGround);

drawFoot(p1,qf1,p.r,p.d,colorFootOne,1,phiBnd);
drawLeg([x;y],ql1,p.l,colorLegOne);

drawFoot(p2,qf2,p.r,p.d,colorFootTwo,2,phiBnd);
drawLeg([x;y],ql2,p.l,colorLegTwo);

plot(p1(1),p1(2),'k.','MarkerSize',SizeJoint);
plot(x,y,'k.','MarkerSize',SizeJoint);
plot(p2(1),p2(2),'k.','MarkerSize',SizeJoint);

axis equal; 

end

function drawFoot(foot,q,r,d,color,id,phiBnd)
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

%Rotation matrix!
R = [cos(q), -sin(q);
    sin(q), cos(q)];

%%% Draw a dashed line showing where the optimization thinks the foot ends:
vc = [0;d];
heel = vc - r*[-sin(phiBnd(1)); cos(phiBnd(1))];
toe = vc - r*[-sin(phiBnd(2)); cos(phiBnd(2))];
dashedLine = R*[heel,vc,toe];
plot(dashedLine(1,:)+foot(1), dashedLine(2,:)+foot(2),'k--');

%%% Draw the virtual circle for the foot:
vc = R*vc;
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