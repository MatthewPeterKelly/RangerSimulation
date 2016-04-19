function info = getStepInfo(stepData, model)
% info = getStepInfo(stepData, model)
%
% This function takes the raw data from the simulation of a single step,
% and computes scalar values that might be of interest for analysis.
%

% Limits on the allowable contact angles
maxContactAngle = 0.3;  %(Heel)
minContactAngle = -0.9;  %(Toe)


% Allocate memory for struct:
info.meanVelX = 0.0;
info.meanVelY = 0.0;
info.meanVelX = 0.0;
info.CoT = 0.0;
info.check.contactRegion_0 = false;
info.check.contactRegion_1 = false;
info.check.ankleLimit_0 = false;
info.check.ankleLimit_1 = false;
info.check.exitFlag = 0;
info.success = false;

% Step duration
info.duration = stepData.t(end) - stepData.t(1);

% Basic kinematics
GI = getAbsoluteCoM(stepData.x(:,1), model.dyn);
GF = getAbsoluteCoM(stepData.x(:,end), model.dyn);
delG = GF - GI;
info.delPosX = delG(1);
info.delPosY = delG(2);
info.meanVelX = info.delPosX/info.duration;
info.meanVelY = info.delPosY/info.duration;

% Cost of transport:
dist = abs(info.delPosX);
weight = 2*model.dyn.m*model.dyn.g;
power = sum(stepData.power) + model.motor.overheadPower;
energy = model.dyn.dt*trapz(power);
info.CoT = (energy/(dist*weight));

% Check the contact region on the foot:
phi0 = stepData.x(3,stepData.k(1,:));  %Foot angles when foot in contact
if isempty(phi0)
    check0 = true;
else
    check0 = max(phi0) < maxContactAngle && min(phi0) > minContactAngle;
end
phi1 = stepData.x(4,stepData.k(2,:));  %Foot angles when foot in contact
if isempty(phi1);
    check1 = true;
else
    check1 = max(phi1) < maxContactAngle && min(phi1) > minContactAngle;
end
checkContact = check0 && check1;
info.check.contactRegion_0 = check0;
info.check.contactRegion_1 = check1;

% Check that the ankle limits are satisfied:
angRel = getAngRel(stepData.x(3:6,:),[],model.dyn.Phi);
q0 = angRel(3,:);
q1 = angRel(4,:);
check0 = min(q0) > 0;
check1 = min(q1) > 0;
checkLimits = check0 && check1;
info.check.ankleLimit_0 = check0;
info.check.ankleLimit_1 = check1;
info.check.exitFlag = stepData.exitFlag;

% Termination condition:
info.success = (checkContact && checkLimits) && stepData.exitFlag == 1;  %Did the step complete successfully?

end