function traj = formatOutput_ds(output)
% 
% This function takes the output of GPOPS and turns it into a structure
% that is useful for data analysis, control, animation, and plotting.
%

config = output.result.setup.auxdata;

soln = output.result.interpsolution.phase;

traj.config = config;

traj.phase(1).t = soln.time';
traj.phase(1).z = soln.state(:,1:4)';
traj.phase(1).dz = soln.state(:,5:8)';
traj.phase(1).u = soln.control';

[pHip, p1, p2] = getJointPos_ds(traj.phase(1).z, config.dyn);
traj.phase(1).pHip = pHip;
traj.phase(1).p1 = p1;
traj.phase(1).p2 = p2;

[dpHip, dp1, dp2] = getJointVel_ds(...
    traj.phase(1).z, traj.phase(1).dz, config.dyn);
traj.phase(1).dpHip = dpHip;
traj.phase(1).dp1 = dp1;
traj.phase(1).dp2 = dp2;

[~,traj.phase(1).f] = dynamics_ds(...
    traj.phase(1).z, traj.phase(1).dz, ...
    traj.phase(1).u, config.dyn);

end