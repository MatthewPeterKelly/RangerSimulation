function decVars = packDecVars(gains, param, scale)
% decVars = packDecVars(gains, param, scale)
%
% INPUTS:
%   gains = struct of controller gains
%   param = struct of controller parameters
%   scale = scale of the perturbations / noise to be optimized
%
% OUTPUTS:
%   decVars = vector of decision variables for optimization
%

decVars = [...
    gains.hip_kp;
    gains.hip_kd;
    gains.ank_push_kp;
    gains.ank_push_kd;
    gains.ank_hold_kp;
    gains.ank_hold_kd;
    gains.ank_flip_kp;
    gains.ank_flip_kd;
    param.ankRefHold;
    param.ankRefFlip;
    param.ankRefPush;
    param.scissorOffset;
    param.scissorRate;
    param.transAngle;
    param.hipRefHold;
    scale];

end