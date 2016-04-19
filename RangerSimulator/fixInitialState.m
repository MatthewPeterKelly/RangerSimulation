function zFixed = fixInitialState(z,p)
%
% This function will adjust the state by vertical translation such that at
% least one foot is in contact with the ground, and both feet are at or
% above the ground.
%

% Figure out the contact angle for each foot:
[p1,p2] = kinematics(z,[],[],p);
qf1 = z(3);  qf2 = z(4);
[qc1, rc1] = getContactInfo(p1,qf1,p);
[qc2, rc2] = getContactInfo(p2,qf2,p);
qc = [qc1;qc2]; rc = [rc1;rc2];

% Determine the kinematics of each contact point
[c1,c2] = kinematicsContact(z,[],qc,rc,p);

% Compute the ground height and normal vector at each contact:
g1 = groundModel(c1(1),p.ground);
g2 = groundModel(c2(1),p.ground);

%Compute the height of each foot:
h1 = c1(2) - g1;
h2 = c2(2) - g2;

%Compute the required vertical shift:
shift = min(h1,h2);  
zFixed = z;
zFixed(2) = zFixed(2) - shift;

end