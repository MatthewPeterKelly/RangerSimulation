function fallen = checkFall(x,dyn)

hipHeight = x(2);
legAng0 = x(5);
legAng1 = x(6);

fallen = false;  %Assume all is good!
if hipHeight < 0.85*dyn.l
    fallen = true;
elseif abs(legAng0) > pi/2
    fallen = true;
elseif abs(legAng1) > pi/2
    fallen = true;
end

end