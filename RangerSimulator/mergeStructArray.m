function output = mergeStructArray(stepData)
% Converts the stepData struct array into a single struct.
%
%

output.exitFlag = stepData(end).exitFlag;
nStep = length(stepData);

% Count entries and shift time
nData = 1;
smallTime = 1e-6;  %Makes plotting and graphics work nicely
for i=1:nStep
    nData = nData + length(stepData(i).t) - 1;
    if i>1
        % dt = new start - old finish
        shift =  stepData(i-1).t(end) + smallTime - stepData(i).t(1);
        stepData(i).t = stepData(i).t + shift;
    end
end

% Initialize memory:
output.t = zeros(1,nData);
output.x = zeros(12,nData);
output.u = zeros(3,nData);
output.f = zeros(4,nData);
output.c = zeros(4,nData);
output.k = zeros(2,nData);
output.y = zeros(10,nData);
output.est = zeros(10,nData);
output.uCmd = zeros(3,nData);
output.qRef = zeros(3,nData);
output.dqRef = zeros(3,nData);
output.fsmMode = zeros(1,nData);
output.kp = zeros(3,nData);
output.kd = zeros(3,nData);
output.uRef = zeros(3,nData);
output.iRef = zeros(3,nData);
output.cp = zeros(3,nData);
output.cd = zeros(3,nData);
output.power = zeros(3,nData);
output.current = zeros(3,nData);
output.uIdeal = zeros(3,nData);
output.dist = zeros(2,nData);

% Copy:
wIdxUpp = 0;
for i=1:nStep
    
    if i == 1
        rIdxLow = 1;
    else
        rIdxLow = 2;
    end
    rIdxUpp = length(stepData(i).t);
    rIdx = rIdxLow:rIdxUpp;
    
    wIdxLow = wIdxUpp + 1;
    if i == 1
        wIdxUpp = wIdxLow + rIdxUpp - 1;
    else
        wIdxUpp = wIdxLow + rIdxUpp - 2;
    end
    wIdx = wIdxLow:wIdxUpp;
    
    output.t(wIdx) = stepData(i).t(rIdx);
    output.x(:,wIdx) = stepData(i).x(:,rIdx);
    output.u(:,wIdx) = stepData(i).u(:,rIdx);
    output.f(:,wIdx) = stepData(i).f(:,rIdx);
    output.c(:,wIdx) = stepData(i).c(:,rIdx);
    output.k(:,wIdx) = stepData(i).k(:,rIdx);
    output.y(:,wIdx) = stepData(i).y(:,rIdx);
    output.est(:,wIdx) = stepData(i).est(:,rIdx);
    output.uCmd(:,wIdx) = stepData(i).uCmd(:,rIdx);
    output.qRef(:,wIdx) = stepData(i).qRef(:,rIdx);
    output.dqRef(:,wIdx) = stepData(i).dqRef(:,rIdx);
    output.fsmMode(:,wIdx) = stepData(i).fsmMode(:,rIdx);
    output.kp(:,wIdx) = stepData(i).kp(:,rIdx);
    output.kd(:,wIdx) = stepData(i).kd(:,rIdx);
    output.uRef(:,wIdx) = stepData(i).uRef(:,rIdx);
    output.iRef(:,wIdx) = stepData(i).iRef(:,rIdx);
    output.cp(:,wIdx) = stepData(i).cp(:,rIdx);
    output.cd(:,wIdx) = stepData(i).cd(:,rIdx);
    output.power(:,wIdx) = stepData(i).power(:,rIdx);
    output.current(:,wIdx) = stepData(i).current(:,rIdx);
    output.uIdeal(:,wIdx) = stepData(i).uIdeal(:,rIdx);
    output.dist(:,wIdx) = stepData(i).dist(:,rIdx);
    
end

end