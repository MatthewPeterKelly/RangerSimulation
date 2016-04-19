function [gains, param, scale] = unpackDecVars(decVars)
% [gains, param, scale] = unpackDecVars(decVars)
%
% INPUTS:
%   decVars = vector of decision variables for optimization
%
% OUTPUTS:
%   gains = struct of controller gains
%   param = struct of controller parameters
%   scale = scale of the perturbations / noise to be optimized
%
%

    gains.hip_kp = decVars(1);
    gains.hip_kd = decVars(2);
    gains.ank_push_kp = decVars(3);
    gains.ank_push_kd = decVars(4);
    gains.ank_hold_kp = decVars(5);
    gains.ank_hold_kd = decVars(6);
    gains.ank_flip_kp = decVars(7);
    gains.ank_flip_kd = decVars(8);

    param.ankRefHold = decVars(9);
    param.ankRefFlip = decVars(10);
    param.ankRefPush = decVars(11);
    param.scissorOffset = decVars(12);
    param.scissorRate = decVars(13);
    param.transAngle = decVars(14);
    param.hipRefHold = decVars(15);
    
    scale = decVars(16);

end