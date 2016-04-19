% Derive_SmoothSaturation.m
%
% Derive the form of the smooth saturation pd controller, such that it can
% be passed to the low-level motor controller.
%

syms kp kd 'real'
syms x v 'real'   %Measured in real time
syms xRef vRef 'real'   % Target set poitns
syms x0 v0 'real'   % Linearize the controller about this point
syms uMax 'real'
syms err 'real'  %Nominal controller output
PI = sym(pi);

u = uMax*tanh(err/uMax);
S = diff(u,err);
err = (kp*(xRef-x0) + kd*(vRef-v0))/uMax;


ux = -S*diff(err,x0);
uv = -S*diff(err,v0);

% uLin = u + ux*(x-x0) + uv*(v-v0);
U0 = simplify(u - ux*x0 - uv*v0);
KP = -ux;
KD = -uv;
