% Derive_Gravity_Compensator
%
% Model ranger's stance foot, leg, and swing leg as a triple pendulum.
%
% Compute the ankle and hip torque to produce a desired acceleration for
% the foot and swing leg joints.
%
% The hope is that this can help to prevent the substantial coupling
% between the hip motor and ankle angle.
%



syms q0 q1 q2 'real'  %Angles of the stance foot, stance leg, and swing foot
syms dq0 dq1 dq2 'real'  %Angular Rates
syms ddq0 ddq1 ddq2 'real'  %Angular acceleration
syms l c r 'real' %lengths of the leg, offset to CoM, and instantaneous foot length
syms m g I 'real' %leg mass, gravity
syms u1 'real' % Torque acting on the stance foot (from the stance leg)
syms uHip 'real' % Torque acting on the swing leg (from the stance leg)

%%%% Unit vectors:

i = sym([1;0]);
j = sym([0;1]);

e0 = cos(q0)*(j) + sin(q0)*(-i);
e1 = cos(q1)*(j) + sin(q1)*(-i);
e2 = cos(q2)*(-j) + sin(q2)*(i);

%%%% State:
z = [q0;q1;q2;dq0;dq1;dq2];
dz = [dq0;dq1;dq2;ddq0;ddq1;ddq2];

%%%% Position vectors:
p0 = sym([0;0]);
p1 = r*e0;  %Ankle one joint
p2 = p1 + l*e1;  %Hip joint
p3 = p2 + l*e2;  %Swing foot
g1 = p2 - c*e1;  % Leg one CoM
g2 = p2 + c*e2;  % Leg two CoM

%%%% Velocity:
dg1 = jacobian(g1,z)*dz;
dg2 = jacobian(g2,z)*dz;

%%%% Acceleration:
ddg1 = jacobian(dg1,z)*dz;
ddg2 = jacobian(dg2,z)*dz;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             DYNAMICS                                    %      
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Define a function for doing '2d' cross product: dot(cross(a,b), k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

%%%% Angular momentum of entire system about contact point:
Sum_Torques_0 = cross2d(g1-p0, -m*g*j) + cross2d(g2-p0, -m*g*j);
Sum_Inertia_0 = cross2d(g1-p0, m*ddg1) + cross2d(g2-p0, m*ddg2) + I*ddq1 + I*ddq2;

%%%% Angular momentum of both legs about ankle joint
Sum_Torques_1 = cross2d(g1-p1, -m*g*j) + cross2d(g2-p1, -m*g*j) - u1;
Sum_Inertia_1 = cross2d(g1-p1, m*ddg1) + cross2d(g2-p1, m*ddg2) + I*ddq1 + I*ddq2;

%%%% Angular momentum of swing leg about hip joint
Sum_Torques_2 = cross2d(g1-p2, -m*g*j) + cross2d(g2-p2, -m*g*j) + uHip;
Sum_Inertia_2 = cross2d(g1-p2, m*ddg1) + cross2d(g2-p2, m*ddg2) + I*ddq1 + I*ddq2;

%%%% Unknowns:
vars = [u1; ddq1; uHip];
eqns = [...
    Sum_Torques_0 - Sum_Inertia_0;
    Sum_Torques_1 - Sum_Inertia_1;
    Sum_Torques_2 - Sum_Inertia_2];

%%%% Create linear system:
[A,b] = equationsToMatrix(eqns,vars);

%%%% Write function:
MM = simplify(subs(A,{'ddq0','ddq2'},{sym(0),sym(0)}));  % Swing leg torque
ff = simplify(subs(b,{'ddq0','ddq2'},{sym(0),sym(0)}));
matlabFunction(MM,ff,'file','autoGen_gravityCompensation.m',...
    'vars',{q0,q1,q2,dq0,dq1,dq2,l,c,r,m,g,I},...
    'outputs',{'MM','ff'});


