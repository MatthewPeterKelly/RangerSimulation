function traj = footFlipTraj(p)

% This function computes the trajectory that the foot flip should follow. 

n = p.n;

if mod(n+1,2)~=0
   error('number of grid points must be positive'); 
elseif n < 7
    error('number of grid points must be at least 7');
end

d = [0,1];
x = chebyshevPoints(n,d);
nMid = (n+1)/2;

%Compute weights for
xList = [0,0.5,1];
wList = [0,1,0];
w = interp1(xList',wList',x').^p.shape;


%%%% Linear Equality constraint:
Aeq = zeros(6,n);   Beq = zeros(6,1);

% Curve must pass through desired points:
Aeq(1,1) = 1;       Beq(1) = p.low;
Aeq(2,nMid) = 1;    Beq(2) = p.mid;
Aeq(3,end) = 1;     Beq(3) = p.upp;

% slope at each point must be equal to zero:
D = chebyshevDifferentiationMatrix(n,d);
Aeq(4:6,:) = D([1,nMid,end],:);


%%%% Set up problem:

options = optimset(...
    'Display','off',...
    'TolFun',1e-3);

yList = [p.low, p.mid, p.upp];

problem.objective = @(yy)( objFun(yy, w,d) );
problem.Aineq = [];
problem.bineq = [];
problem.Aeq = Aeq;
problem.beq = Beq;
problem.lb = min(yList)*ones(1,n) - 1e-3;
problem.ub = max(yList)*ones(1,n) + 1e-3;
problem.x0 = 0.5*(problem.lb+problem.ub);
problem.nonlcon = [];
problem.options = options;
problem.solver = 'fmincon';

% Solve using fmincon
[y,~,exitFlag] = fmincon(problem);

if exitFlag ~= 1
    disp('WARNING: foot flip trajectory did not converge')
end

% Return the trajectory:
traj.time = x;
traj.angle = y;
traj.domain = d;

end

function cost = objFun(y,w,d)
% Computes a weighted minimal-jerk trajectory

% Compute curvature
[~,ddy] = chebyshevDerivative(y,d); 

% weighted sum of squares:
cost = sum(dot(w,ddy.^2));

end



