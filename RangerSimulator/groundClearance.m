function [h, r] = groundClearance(th,p,q,param)
%
% This function is used inside of collision detection. It helps determine
% the point on the foot that is penetrating the ground the most.
%
% INPUTS:
%    th = absolute angle of contact point on foot
%    p = [x;y] = location of the ankle joint
%    q = ansolute angle of the foot
%
% OUTPUTS:
%    h = penetration between the chosen contact point on the foot and the
%    ground model.
%

% Find the radius of the foot in foot reference frame
r = footModel(th - q);

[cy, cx] = pol2cart(th,r); cy = -cy;  %Compute the height of the contact point
cx = cx + p(1); cy = cy + p(2);

gy = groundModel(cx,param);  %Compute the hight of the ground here

h = cy-gy;  %Return the penetration:

end