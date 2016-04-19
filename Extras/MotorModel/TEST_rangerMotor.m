% TEST_rangerMotor.m
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

Param.smoothing = 0.01;

omega_list = linspace(-0.3,0.3,100);
I_list = linspace(-8,8,9);

[omega, I] = meshgrid(omega_list, I_list);

[T, P] = RangerMotor(omega, 0, I, Param);

[Ts, Ps] = RangerMotorSmooth(omega, I, Param);

figure(100); clf;

subplot(1,2,1); hold on;
plot(omega_list,T,'k.')
plot(omega_list,Ts)
xlabel('gearbox output speed');
ylabel('Torque')
title('torque for a variety of currents')

subplot(1,2,2); hold on;
plot(omega_list,P,'k.')
plot(omega_list,Ps)
xlabel('gearbox output speed');
ylabel('Power')
title('power for a variety of currents')

