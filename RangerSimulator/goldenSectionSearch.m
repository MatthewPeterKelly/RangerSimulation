function [x, fx] = goldenSectionSearch(xLow,xUpp,maxIter,tol)
% [xMin, yMin] = goldenSectionSearch(xLow,xUpp,maxIter,tol)
%
% Solves a a scalar non-linear optimization problem, using Matlab's fminbnd
% algorithm, without the added overheads. Designed to be used inside of
% simulations that are being compiled to mex.
%
% MEX does not support function handles, so this optimization routine has
% the function name hard coded into it as OBJECTIVE_FUNCTION
%
% INPUTS:
%   OBJECTIVE_FUNCTION - function handle y = f(x)
%   xLow - lower search bound
%   xUpp - upper search bound
%   maxIter - maximum allowed iterations
%
% OUTPUTS:
%   xMin - location of the minimum
%   yMin - function value at minimum
%   exitFlag - 0=maxIter, 1=success
%

% Compute the start point
seps = sqrt(eps);
c = 0.5*(3.0 - sqrt(5.0));
a = xLow; b = xUpp;
v = a + c*(b-a);
w = v; xf = v;
d = 0.0; e = 0.0;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               CALL TO THE HARD-CODED OBJECTIVE FUNCTION                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
x= xf; fx = OBJECTIVE_FUNCTION(x);    %%%% CHANGE THIS LINE %%%%

fv = fx; fw = fx;
xm = 0.5*(a+b);
tol1 = seps*abs(xf) + tol/3.0;
tol2 = 2.0*tol1;

% Main loop
iter = 0;
while ( abs(xf-xm) > (tol2 - 0.5*(b-a)) )
    gs = 1;
    % Is a parabolic fit possible
    if abs(e) > tol1
        % Yes, so fit parabola
        gs = 0;
        r = (xf-w)*(fx-fv);
        q = (xf-v)*(fx-fw);
        p = (xf-v)*q-(xf-w)*r;
        q = 2.0*(q-r);
        if q > 0.0,  p = -p; end
        q = abs(q);
        r = e;  e = d;
        
        % Is the parabola acceptable
        if ( (abs(p)<abs(0.5*q*r)) && (p>q*(a-xf)) && (p<q*(b-xf)) )
            
            % Yes, parabolic interpolation step
            d = p/q;
            x = xf+d;
            
            % f must not be evaluated too close to ax or bx
            if ((x-a) < tol2) || ((b-x) < tol2)
                si = sign(xm-xf) + ((xm-xf) == 0);
                d = tol1*si;
            end
        else
            % Not acceptable, must do a golden section step
            gs=1;
        end
    end
    if gs
        % A golden-section step is required
        if xf >= xm, e = a-xf;    else e = b-xf;  end
        d = c*e;
    end
    
    % The function must not be evaluated too close to xf
    si = sign(d) + (d == 0);
    x = xf + si * max( abs(d), tol1 );
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %           CALL TO THE HARD-CODED OBJECTIVE FUNCTION                 %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    fu = OBJECTIVE_FUNCTION(x);   %%%% CHANGE THIS LINE %%%%
    
    iter = iter + 1;
    
    % Update a, b, v, w, x, xm, tol1, tol2
    if fu <= fx
        if x >= xf, a = xf; else b = xf; end
        v = w; fv = fw;
        w = xf; fw = fx;
        xf = x; fx = fu;
    else % fu > fx
        if x < xf, a = x; else b = x; end
        if ( (fu <= fw) || (w == xf) )
            v = w; fv = fw;
            w = x; fw = fu;
        elseif ( (fu <= fv) || (v == xf) || (v == w) )
            v = x; fv = fu;
        end
    end
    xm = 0.5*(a+b);
    tol1 = seps*abs(xf) + tol/3.0; tol2 = 2.0*tol1;
    
    if iter >= maxIter
        % Failed to converge - return quietly
        return
    end
end % while

end