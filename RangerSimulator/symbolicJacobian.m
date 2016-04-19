function [Mx, Mu, Bx, Bu] = symbolicJacobian(M,B,x,u)
%  [Mx, Mu, Bx, Bu] = symbolicJacobian(M,B,x,u)
%
%  This function computes the jacobian for systems where the dynamics are
%  defined using a numerical solve on each time step:
%
%   dx = M(x,u)\B(x,u) = f(x,u)
%
%   Let the linearized system be:
%
%   f = (df/dx)*xDel + (df/du)*uDel + fRef(xRef, uRef)
%
%   Then the jacobian (df/dx) and (df/du) are slightly trickey...
%
%  df/dx = -(M\(dM/dx)) * 

%http://planetmath.org/derivativeofinversematrix


error('Incomplete!')

end