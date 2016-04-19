function r = footModel(th)
% r = footModel(th)
%
% This function computes the distance (r) from the ankle joint to the edge
% of the foot, given some angle (th) in the frame of the foot.
%
% INPUTS:
%   th = [1, n] = vector of query angles, measured in the frame of the
%   foot, where 0 corresponds to the -j axis. For example:
%
%           [y,x] = pol2cart(th,r); y = -y;
%
% OUTPUTS:
%   r = [1, n] = radius (edge of foot to axle) in meters
%
% NOTES:
%   --> The foot is modeled using a piecewise 5th-order polynomial that has
%   continuous second derivatives across each knot point, and is periodic.
%

%%%% Parameters computed in MAIN_circleModel.m

% Knot points for the piecewise 5th-order polynomial
Th = [...
      -1.570796326794897;
  -0.561694831717942;
  -0.286174659654303;
   1.250200489359516;
   1.466572167779030;
   2.034443935795703;
   4.712388980384690];
    
% Function value, slope, and curvature at knot points:
% [r(Th), dr(Th), ddr(Th)];
P = [...
   0.026429428615028   0.001120783999919   0.003819397812255;
   0.049616197502626   0.077679469811930   0.263409043236919;
   0.061753405058648  -0.012561666565018   0.047665562305856;
   0.105377371861968   0.094066800804457   0.146386617183001;
   0.088455248234482  -0.220297541073049   1.089533124700152;
   0.040233205117705  -0.030857935325635   0.019752990773765;
   0.026429428615028   0.001120783999919   0.003819397812255];

% Map th to the appropriate domain:
thWrap = wrapTo2Pi(th - Th(1)) + Th(1);

% Evaluate the piecewise 5th-order curve:
r = pwPoly5(Th',P',thWrap);

end

