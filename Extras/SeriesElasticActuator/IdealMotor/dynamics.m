function dz = dynamics(z,u,P)

qf = z(1,:);
qm = z(2,:);
dqf = z(3,:);
dqm = z(4,:);

Tm = u;  %Motor torque

[ddqm,ddqf] = autoGen_dynamics(qm,dqm,qf,dqf,Tm,P.Jm,P.Ks,P.Jf,P.Kf);

dz = [dqf; dqm; ddqf;ddqm];

end