function [th, fx, radius] = findContactPoint(thLow,thUpp,maxIter,tol,pAnkle,qf,param)
% [x, fx, radius] = findContactPoint(thLow,thUpp,maxIter,tol,pAnkle,footRadius,param)
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
%   thLow - lower search bound
%   thUpp - upper search bound
%   maxIter - maximum allowed iterations
%
% OUTPUTS:
%   x - location of the minimum
%   yMin - function value at minimum
%   exitFlag - 0=maxIter, 1=success
%

% Compute the start point
seps = sqrt(eps);
c = 0.5*(3.0 - sqrt(5.0));
a = thLow; b = thUpp;
v = a + c*(b-a);
w = v; thF = v;
d = 0.0; e = 0.0;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               CALL TO THE HARD-CODED OBJECTIVE FUNCTION                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
th = thF; [fx, radius] = groundClearance(th,pAnkle,qf,param);    

fv = fx; fw = fx;
xm = 0.5*(a+b);
tol1 = seps*abs(thF) + tol/3.0;
tol2 = 2.0*tol1;

% Main loop
iter = 0;
while ( abs(thF-xm) > (tol2 - 0.5*(b-a)) )
    gs = 1;
    % Is a parabolic fit possible
    if abs(e) > tol1
        % Yes, so fit parabola
        gs = 0;
        r = (thF-w)*(fx-fv);
        q = (thF-v)*(fx-fw);
        p = (thF-v)*q-(thF-w)*r;
        q = 2.0*(q-r);
        if q > 0.0,  p = -p; end
        q = abs(q);
        r = e;  e = d;
        
        % Is the parabola acceptable
        if ( (abs(p)<abs(0.5*q*r)) && (p>q*(a-thF)) && (p<q*(b-thF)) )
            
            % Yes, parabolic interpolation step
            d = p/q;
            th = thF+d;
            
            % f must not be evaluated too close to ax or bx
            if ((th-a) < tol2) || ((b-th) < tol2)
                si = sign(xm-thF) + ((xm-thF) == 0);
                d = tol1*si;
            end
        else
            % Not acceptable, must do a golden section step
            gs=1;
        end
    end
    if gs
        % A golden-section step is required
        if thF >= xm, e = a-thF;    else e = b-thF;  end
        d = c*e;
    end
    
    % The function must not be evaluated too close to thF
    si = sign(d) + (d == 0);
    th = thF + si * max( abs(d), tol1 );
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %           CALL TO THE HARD-CODED OBJECTIVE FUNCTION                 %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    [fu, radius] = groundClearance(th,pAnkle,qf,param);  
    
    iter = iter + 1;
    
    % Update a, b, v, w, x, xm, tol1, tol2
    if fu <= fx
        if th >= thF, a = thF; else b = thF; end
        v = w; fv = fw;
        w = thF; fw = fx;
        thF = th; fx = fu;
    else % fu > fx
        if th < thF, a = th; else b = th; end
        if ( (fu <= fw) || (w == thF) )
            v = w; fv = fw;
            w = th; fw = fu;
        elseif ( (fu <= fv) || (v == thF) || (v == w) )
            v = th; fv = fu;
        end
    end
    xm = 0.5*(a+b);
    tol1 = seps*abs(thF) + tol/3.0; tol2 = 2.0*tol1;
    
    if iter >= maxIter
        % Failed to converge - return quietly. "Good enough" is fine here.
        return
    end
end % while

end