function [y,n] = groundModel(x,p)
% [y,n] = groundModel(x,p)
%
% This function returns the height (y) and normal vector (n) of the ground
% model at the horizontal position x.
%
% INPUTS:
%   x = scalar horizontal position
%   p = [1, 6] parameter vector
%
% OUTPUTS:
%   y = scalar ground height
%   n = [2, 1] ground normal vector
%
% NOTES:
%   y = p1 + p2*x + p3*sin(2*pi*(p4 *x + p5)) + p6*x.^2;
%

y = p(1) + p(2)*x + ...     % Linear
    p(3)*sin(2*pi*(p(4)*x+p(5))) ...  %Sine
    + p(6)*x.*x;   % Quadratic
if nargout > 1
    dy = p(2) + ...
        p(3)*p(4)*2*pi*cos(2*pi*(p(4)*x+p(5))) + ...
        2*p(6)*x;
    n = [-dy;1]/sqrt(1+dy*dy);
end

end