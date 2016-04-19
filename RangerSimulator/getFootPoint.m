function [y,x,r] = getFootPoint(qFoot,qTest)
% [y,x] = getFootPoint(qFoot,qTest)
%
% This function computes a point on the foot (x,y) given the absolute angle
% of the foot (qFoot) and an absolute angle of the query point in the world
% frame (qTest). The point is given with respect to the ankle joint.
%

r = footModel(qTest-qFoot);

x = r*sin(qTest);
y = -r*cos(qTest);

end