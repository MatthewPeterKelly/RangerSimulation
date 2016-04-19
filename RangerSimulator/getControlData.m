function controlData = getControlData()

%%%% Controller Parameters, Identical to real robot: %%%%%%%%%%%%%%%%%%%%%%
controlData.walk.hip_kp = 25;
controlData.walk.hip_kd = 3;
controlData.walk.ank_push_kp = 20;
controlData.walk.ank_push_kd = 2;
controlData.walk.ank_stance_kp = 15;
controlData.walk.ank_stance_kd = 1;
controlData.walk.ank_swing_kp = 20;
controlData.walk.ank_swing_kd = 1;
controlData.walk.ank_push = 0.8;  %Normalized push-off angle
controlData.walk.crit_step_length = 0.2;
controlData.walk.scissor_gain = 1.2;
controlData.walk.scissor_offset = 0.1;
controlData.walk.doubleStance_duration = 0.08;   %How long to continue push-off after heel-strike
%
%
controlData.walk.ank_flipTarget = 0.2;   % relative ankle angle when foot is flipped up
controlData.walk.ank_holdLevel = 0.0;   %absolute foot angle target for hold level
controlData.walk.ank_pushTarget = -1.0;  %absolute foot angle target for push-off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end