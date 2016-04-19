function u = controller(q0,dq0,kp,kd)
% u = controller(q0,q1,dq0,dq1,kp,kd)
%
% This controller attempts to hold the foot level while the pendulum
% rotates around it.
%

u = kp*(0 - q0) + kd*(0 - dq0);

end