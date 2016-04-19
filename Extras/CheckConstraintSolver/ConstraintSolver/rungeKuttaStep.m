function zNext = rungeKuttaStep(h,z,xi,wn)

k1 = simpleDynamics(z,xi,wn);
k2 = simpleDynamics(z+0.5*h*k1,xi,wn);
k3 = simpleDynamics(z+0.5*h*k2,xi,wn);
k4 = simpleDynamics(z+h*k3,xi,wn);

zNext = z + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

end