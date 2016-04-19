function traj = buildRefTraj(traj)
% traj = buildRefTraj(traj)
%
% This function computes the piecewise polynomial trajectories that make up
% the reference trajectory for the feedback linearization
%
% INPUTS:
%   traj.phase = fine-grid trajectories:
%           .t = time
%           .z = configuration
%           .dz = dz/dt
%           .ddz = ddz/ddt
%   traj.ref = prototype for ref struct, with fields:
%       .nGrid = number of points to store reference trajectory at
%       .wn = natural frequency of the controller
%       .xi = damping ratio of the controller
%       .c = [1, nConfig] = mapping from configuration to phase
%       .H = [nMeasure, nConfig] = mapping from configuration to measurement
%
% OUTPUTS:
%
%   traj.ref = full ref struct, with added fields:
%       .pp = piecewise-polynomial trajectories for:
%           .h = reference measurement vector
%           .dh = dMeasurement/dPhase
%           .ddh = second derivative of measurements with respect to phase 
%           .dhdt = dMeasurement/dTime
%       
%

t = linspace(traj.phase.t(1),traj.phase.t(end),traj.ref.nGrid);
q = interp1(traj.phase.t', traj.phase.z', t','pchip')';
dq = interp1(traj.phase.t', traj.phase.dz', t','pchip')';
ddq = interp1(traj.phase.t', traj.phase.ddz', t','pchip')';

% Trajectories: measurement and phase vs time:
hRef = traj.ref.H*q;   % Target measurement
dhRef = traj.ref.H*dq;
ddhRef = traj.ref.H*ddq;
pRef = traj.ref.c*q; % Phase
dpRef = traj.ref.c*dq;
ddpRef = traj.ref.c*ddq;


% Compute derivatives wrt phase using chain rule:
nMeasure = size(hRef,1);
one = ones(nMeasure,1);
dhRefdp = dhRef./(one*dpRef);
ddhRefddp = (ddhRef - dhRefdp.*(one*ddpRef))./((one*dpRef).^2);

% Represent trajectories as piecewise-polynomial
traj.ref.pp.h = pchip(pRef,hRef);
traj.ref.pp.dh = pchip(pRef,dhRefdp);   % Derivative wrt phase  (for ref traj)
traj.ref.pp.dhdt = pchip(pRef,dhRef);   % Derivative wrt time   (for stabilization)
traj.ref.pp.ddh = pchip(pRef,ddhRefddp);

end