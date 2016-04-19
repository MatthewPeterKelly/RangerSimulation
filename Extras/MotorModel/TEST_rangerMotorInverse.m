% TEST_rangerMotorInverse.m
%
% This script is used to test the motor model function:

Param.R = 1.3; % ohms (terminal resistance)
Param.Vc = 0.7; % volts (contact voltage drop)
Param.K = 0.018; % Nm/A (torque constant)
Param.C1 = 0; % Nms/rad (viscous friction)
Param.C0 = 0.01; % Nm (constant friction)
Param.mu = 0.1; % (current-depentent constant friction)
Param.Jm = 1.6e-6; % kg m^2 (motor inertia)
Param.G = 66; % {66 -> hip, 34 -> ankle} gearbox ratio

nTest = 1000;
omega = 80*(pi/180)*(1-2*rand(nTest,1));
alpha = 0*(pi/180)*(1-2*rand(nTest,1));
I = 8*(1-2*rand(nTest,1));

T = RangerMotor(omega, alpha, I, Param);

[Icheck, flag] = RangerMotorInverse(omega, alpha, T, Param);

err = I-Icheck;
disp(['Max Error: ' num2str(max(abs(err)))]);

% for i=1:nTest
%    fprintf('Flag: %d, error: %e\n',flag(i),err(i)); 
% end