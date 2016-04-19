function a = getAction(v,A,V)
%
% This function interpolates the action matrix to get the action at the
% desire speed.
%

% Clamp to valid domain:
if v > V(end), v = V(end); end;
if v < V(1), v = V(1); end;

% Linear interpolation:
a = interp1(V',A',v,'linear')';

end