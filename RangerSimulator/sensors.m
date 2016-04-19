function y = sensors(x, k, model_sensors)
% y = sensors(x, model_sensors)
%
% This function is a sensor model, converting the truth state of the
% simulation into something that could have come out of Ranger's sensors.
%
% INPUTS:
%
% x = [z;dz];
%     z = [6, 1] = full configuration vector
%     dz = [6, 1] = full configuration rate vector
%     ddz = [6, 1] = full configuration acceleration vector
%         (1) = hip horizontal position
%         (2) = hip vertical position
%         (3) = outer foot absolute angle
%         (4) = inner foot absolute angle
%         (5) = outer leg absolute angle
%         (6) = inner leg absolute angle
%
% model_sensors = struct with sensor parameters
%
% OUTPUTS:
%   
%   y = [qr;qh;q0;q1; qr;qh;q0;q1];   % sensor data (possibly noisy)
%

angAbs = x(3:6);
rateAbs = x(9:12);
[angRel, rateRel] = getAngRel(angAbs,rateAbs,model_sensors.Phi);

y = [angRel; rateRel; k];

end