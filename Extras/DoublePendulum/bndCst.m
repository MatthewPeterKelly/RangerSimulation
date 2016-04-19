function [c, ceq] = bndCst(t0,x0,tF,xF)

% Force the hip angle rate (q2-q1) to be zero at the end of the step:

c = [];
ceq = xF(4,:)-xF(3,:);


end