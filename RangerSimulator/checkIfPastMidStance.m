function isPast = checkIfPastMidStance(x,k,flagOuter)
% isPast = checkIfPastMidStance(x,k,flagOuter)
%
% This function returns true if the simulation has passed mid-stance on the
% correct foot

if flagOuter   %Waiting for mid-stance on the outer feet
    th = x(5);
    w = x(11);
    c = k(1);   %Outer feet in contact
else   %Waiting for mid-stance on the inner feet
    th = x(6);
    w = x(11);
    c = k(2);   % Inner feet in contact
end

isPast = ...
    (th < 0) && ...  %Must be leaning forward
    (w < 0)  && ...   %Must be falling forward
    c;   %Stance foot must be in contact with the ground
end