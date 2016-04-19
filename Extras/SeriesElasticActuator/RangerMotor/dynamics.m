function ddz = dynamics(z,dz,u,P)

qf = z(1,:);
qm = z(2,:);
ql = z(3,:);

dqf = dz(1,:);
dqm = dz(2,:);
dql = dz(3,:);

Ia = u;  %Motor current

ddz = autoGen_dynamics(...
    qf,qm,ql,dqf,dqm,dql,Ia,...
    P.Rad, P.Len, P.Ecc, P.g, P.Jf, P.Jm, P.Jl, P.m, P.Km, P.Ks, P.Ga, P.C0, P.C1, P.Mu, P.Smooth, P.Cf);

end