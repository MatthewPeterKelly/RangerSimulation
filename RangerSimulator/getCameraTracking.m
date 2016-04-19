function camera = getCameraTracking(output)
% camera = getCameraTracking(output)
%
% This function returns a matrix of points where the first row is the
% horizontal position and the second is the vertical position of the target
% point for the camera in the animation.
%

t = output.t;
x = output.x(1,:);
y = output.x(2,:);

yShift = -0.5*mean(y);

trackingFrequency = 0.5;  %(Hz) 
dataRate = mean(diff(t));
Wn = trackingFrequency*2*dataRate;
[B,A] = butter(2,Wn,'low');

xCam = filtfilt(B,A,x);
yCam = filtfilt(B,A,y) + yShift;

camera = [xCam; yCam];

end