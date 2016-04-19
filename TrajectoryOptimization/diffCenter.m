function dx = diffCenter(x,dt)
% dx = diffCenter(x,dt)
%
%   Computes the second-order finite difference approximation of x with
%   respect to t. A one-sided second order difference is used at the end
%   points, so size(dx) == size(x).
%
%   INPUTS:
%       x = [nState x nTime] vector of function values 
%       dt = sampling period of x   (default = 1)
%
%   OUTPUTS:
%       d = dx/dt = first derivative of x wrt t
%
% See also: cumInt, diff

if nargin == 1
    dt = 1;
end

if length(dt) ~= 1
    error('Time-step (dt) must be a scalar');
end

n = length(x);
if n < 2
    error('length(x) must be at least 2');
elseif n == 2
    dx = diff(x,1,2)/dt;
else %Then enough points for second-order finite difference
    dx = zeros(size(x));
    Dx = diff(x,1,2);   %Matlab first-order finite difference along second dimension
    dx(:,1) = (-3*x(:,1) + 4*x(:,2) - x(:,3))/(2*dt);
    dx(:,2:(end-1)) = (Dx(:,1:(end-1)) + Dx(:,2:end))/(2*dt);  %Mid-points are easy
    dx(:,end) = (x(:,end-2) - 4*x(:,end-1) + 3*x(:,end))/(2*dt);
end

end