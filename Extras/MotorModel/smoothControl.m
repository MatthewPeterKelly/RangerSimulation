function [u, KK] = smoothControl(x,v,P)
% u = smoothControl(x,v,P)
%
% This function computes a controller that continuously switches between a
% pd controller near the origin, and a bang-bang controller when far from
% the origin. This is similar to sliding mode control.
%

kp = P.kp;
kd = P.kd;
uMax = P.uMax;
xRef = P.xRef;
vRef = P.vRef;

err = kp*(xRef-x) + kd*(vRef-v);

u = uMax*tanh(err/uMax);

if nargout == 2
    % The return the linearized form of the controller
    
   ue = 1 - tanh(err/uMax)^2;  %Derivative of u wrt err
   ex = kp;   %derivative of err wrt -x
   ev = kd;   %derivative of err wrt -v
   
   KK.kp = ue*ex;  %local proportional gain
   KK.kd = ue*ev;  %local derivative gain
   KK.u0 = u + KK.kp*x + KK.kd*v;
   KK.info = 'u = u0 - kp*x - kd*v;';
   
end

end