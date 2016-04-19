% Derive the equations of motion for a series elastic actuator
%
%

% Parameters:
syms Jm Ks Jf Kf 'real'

% States:
syms qm dqm ddqm 'real'   %Motor angle
syms qf dqf ddqf 'real'    %Shaft angle at load (after spring)

% Forces:
syms Tm 'real'   %Torque applied by the motor
syms Tf 'real'   %Torque applied at load (after spring)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Basic Equations:
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Motor Eqn:   Tm + Ks*(qf-qm) == Jm*ddqm
eqn1 = Tm + Ks*(qf-qm) - Jm*ddqm;

%%%% Actuator result:
Tf = -Ks*(qf-qm);

%%%% Connect it to a pendulum for now:  Tf = Kf*sin(qf) + Jf*ddqf
eqn2 = Tf - (  Kf*sin(qf) + Jf*ddqf  );


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Solve:
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

eqns = [eqn1; eqn2];
vars = [ddqm; ddqf];
[AA,bb] = equationsToMatrix(eqns, vars);
soln = simplify(AA\bb);

matlabFunction(soln(1), soln(2),...
    'file','autoGen_dynamics.m',...
    'vars',{qm, dqm, qf, dqf, Tm, Jm, Ks, Jf, Kf},...
    'outputs',{'ddqm','ddqf'});

