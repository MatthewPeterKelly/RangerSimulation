function u = ctrlNoOscillation(~,z,xi,P)

% Adds an artificial damping term to achieve a critical damping between the
% motor and the end effector

% qf = z(1,:);
% qm = z(2,:);
dqf = z(3,:);
dqm = z(4,:);


Jm = P.Jm;   % Motor inertia
Ks = P.Ks;  %Spring constant

u = 2*xi*sqrt(Ks*Jm)*(dqf-dqm);

end