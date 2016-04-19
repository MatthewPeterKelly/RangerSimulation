function q = footFlipCst(t,T,thLow,thUpp)
% q = footFlipCst(t,T,thLow,thUpp)
%
% This function computes the minimum angle that the foot can be at, as a
% function of time in the step. This is used to ensure that the swing foot
% clears the ground.
%
% INPUTS:
%   t = time vector 0 < t < T
%   T = step duration 
%   thLow = angle at t = 0 and t=T
%   thUpp = angle at t = T/2;
%
% OUTPUTS:
%   q(t) is quadratic and 
%       q(0) = thLow
%       q(T/2) = thUpp
%       q(T) = thLow
%

q = thUpp - 4*(thUpp-thLow)*(t/T - 0.5).^2;

end