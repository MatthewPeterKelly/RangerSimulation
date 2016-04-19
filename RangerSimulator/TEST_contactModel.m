% Test_contactModel.m
%
% This script tests the contact model, checking to see if it correctly
% predicts the point where the foot touches the ground.
%
%


qFoot = -0.6;  %Angle to test foot contact point.
origin = [0;0];
color = [35, 180, 90]/255;


figure(4); clf; hold on
drawRangerFoot(origin,qFoot,color);


[x,y] = footContact_cart(qFoot);

plot(origin(1)+x, origin(2)+y,'kx','MarkerSize',10,'LineWidth',3);


[r,th] = footContact_pol(qFoot);
plot(origin(1)+r*sin(th), origin(2)-r*cos(th),'ko','MarkerSize',10,'LineWidth',3);