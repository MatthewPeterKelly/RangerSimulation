function d = getPushDisturbance(t,D)
% d = getPushDisturbance(t,D)
%
% This function computes the horizontal and vertical push disturbance,
% given the start time (D.t0) end time (D.t1) and magnitude (D.fx, D.fy) 
% of the push. All inputs are scalar.
%

if D.t0 < t && t < D.t1
    d = [D.fx; D.fy];
else
    d = [0;0];
end

end