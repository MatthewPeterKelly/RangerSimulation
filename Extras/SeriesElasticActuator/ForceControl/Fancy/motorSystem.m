function [dz, I, f, fRef,qEnd,dqEnd] = motorSystem(t,z,P)

q = z(1,:);
dq = z(2,:);

% Target force to achieve
switch P.forceProfile
    case 'freqSweep'
        fRef = 3*sin(3*t.^2);
    case 'squareWave'
        fRef = 3*tanh(3*sin(4*t));
    case 'constant'
        fRef = ones(size(t));
    case 'forceData'
        fRef = interp1(P.data.time',P.data.force',t','pchip')';
    otherwise
        fRef = zeros(size(t));
end

% Angle and velocity of the end effector
switch P.angleProfile
    case 'sineWave'
        a = 1.0;   %Angle that the end effector is driven through
        b = 2*pi*1;
        qEnd = a*sin(b*t);
        dqEnd = a*b*cos(b*t);
    case 'fastSineWave'
        a = 1.0;   %Angle that the end effector is driven through
        b = 2*pi*4;
        qEnd = a*sin(b*t);
        dqEnd = a*b*cos(b*t);
    case 'constant'
        qEnd = zeros(size(t));
        dqEnd = zeros(size(t));
    case 'angleData'
        qEnd = interp1(P.data.time',P.data.angle',t','pchip')';
        dqEnd = interp1(P.data.time',P.data.rate',t','pchip')';
    otherwise
        qEnd = zeros(size(t));
        dqEnd = zeros(size(t));
end

I = motorControl(qEnd,dqEnd,q,dq,fRef,P);
% I = motorControl_oldVersion(qEnd,dqEnd,q,dq,fRef,P);

[ddq, f] = motorDynamics(qEnd,q,dq,I,P);

dz = [dq;ddq];

end

