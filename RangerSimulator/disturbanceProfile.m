function y = disturbanceProfile(t,t0,t1)
% y = disturbanceProfile(t,t0,t1)
%
% Select the shape of the disturbance profile

% Scale time horizon
t = (t-t0)/(t1-t0);

% Sigmoid. y(0) approx 0. y(1) approx 1.
y = 1./(exp(-10*(t-0.5)) + 1);


end
