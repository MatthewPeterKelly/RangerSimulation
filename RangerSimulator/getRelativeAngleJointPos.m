function [delX,delY] = getRelativeAngleJointPos(thStance, thSwing, l)
%
% This function computes the relative position of the stance leg ankle with
% respect to the swing leg ankle
%

pHip = l*[...
    -sin(thStance);
    cos(thStance)];

pFoot = pHip + l*[...
    sin(thSwing);
    -cos(thSwing)];

delX = pFoot(1);
delY = pFoot(2);

end