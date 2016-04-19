function [zFlip, dzFlip] = switchFeet(z,dz)
%
% This function switches the roles of the feet. Foot one becomes foot two
% and foot two becomes foot one. 
%
% INPUTS:
%   z = [6 x n] configuration matrix
%   dz = [6 x n] configuration rate matrix
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

zFlip = [...
    z(1,:);
    z(2,:);
    z(4,:);
    z(3,:);
    z(6,:);
    z(5,:)];

dzFlip = [...
    dz(1,:);
    dz(2,:);
    dz(4,:);
    dz(3,:);
    dz(6,:);
    dz(5,:)];

end