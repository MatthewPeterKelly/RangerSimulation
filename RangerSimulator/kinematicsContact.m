function [p0c,p1c,dp0c,dp1c] = kinematicsContact(z,dz,qc,rc,dyn)
% [p0c,p1c,dp0c,dp1c] = kinematicsContact(z,dz,qc,rc,dyn)
%
% This function computes the kinematics of the contact point on each foot
%
% INPUTS:
% z = [6, 1] = full configuration vector
% dz = [6, 1] = full configuration rate vector
% 	(1) = hip horizontal position
% 	(2) = hip vertical position
% 	(3) = outer foot absolute angle
% 	(4) = inner foot absolute angle
% 	(5) = outer leg absolute angle
% 	(6) = inner leg absolute angle
%
%   qc = [2 x 1] contact point angle relative to foot
%   rc = [2 x 1] contact point radius
%   dyn = parameter struct:
%    .d = eccentricity of the foot
%    .l = leg length (hip joint to foot joint)
%
% OUTPUTS:
%   p0c = position of outer foot contact point
%   p1c = position of inner foot contact point
%   dp0c = velocity of outer foot contact point
%   dp1c = velocity of inner foot contact point
%
%   Angle Conventions:
%       - All angles are in absolute reference frame
%       - zero corresponds to:
%           (legs) = ankle joint directly below hip joint
%           (feet) = ankle joint directly below virtual foot center
%

x = z(1);
y = z(2);
phi0 = z(3);
phi1 = z(4);
th0 = z(5);
th1 = z(6);

qc0 = qc(1);
qc1 = qc(2);
rc0 = rc(1);
rc1 = rc(2);

if nargout > 2  %Then need velocities
    dx = dz(1);
    dy = dz(2);
    dphi0 = dz(3);
    dphi1 = dz(4);
    dth0 = dz(5);
    dth1 = dz(6);
end

if nargout <= 2 % Calculate positions onle
    [p0c,p1c] = autoGen_kinematicsContact(...
        x,y,phi0,phi1,th0,th1,...
        [],[],[],[],[],[],...
        qc0,qc1,rc0,rc1,...
        dyn.l);
elseif nargout > 2 % calculate positions and velocities
    [p0c,p1c,dp0c,dp1c] = autoGen_kinematicsContact(...
        x,y,phi0,phi1,th0,th1,...
        dx,dy,dphi0,dphi1,dth0,dth1,...
        qc0,qc1,rc0,rc1,...
        dyn.l);
end

end