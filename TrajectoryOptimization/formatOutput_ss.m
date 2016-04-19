function traj = formatOutput_ss(output)
% 
% This function takes the output of GPOPS and turns it into a structure
% that is useful for data analysis, control, animation, and plotting.
%

config = output.result.setup.auxdata;

soln = output.result.interpsolution.phase;

traj.config = config;

z = soln.state(:,1:4)';
dz = soln.state(:,5:8)';
u = soln.control';
p = config.dyn;

traj.phase(1).t = soln.time';
traj.phase(1).z = z;
traj.phase(1).dz = dz;
traj.phase(1).u = u;

[pHip, p1, p2, dpHip, dp1, dp2] = kinematics_ss(z, dz, p);
traj.phase(1).pHip = pHip;
traj.phase(1).p1 = p1;
traj.phase(1).p2 = p2;
traj.phase(1).dpHip = dpHip;
traj.phase(1).dp1 = dp1;
traj.phase(1).dp2 = dp2;

ddz = dynamics_ss(z,dz,u,p);
f1 = getContactForce_ss(z,dz,ddz,p);
f2 = zeros(size(f1));
traj.phase(1).f = [f1;f2];
traj.phase(1).ddz = ddz;

end