function [dz, I, f, fRef] = motorSystem(t,z,P)

q = z(1,:);
dq = z(2,:);

switch P.forceProfile
    case 'freqSweep'
        fRef = frequencySweep(t);
    case 'squareWave'
        fRef = squareWave(t);
    otherwise
        fRef = zeros(size(t));
end

I = motorControl(q,dq,fRef,P);
[ddq, f] = motorDynamics(q,dq,I,P);

dz = [dq;ddq];

end


function fRef = frequencySweep(t)

fRef = 4*sin(3*t.^2);  %Simple sine curve for now

end

function fRef = squareWave(t)

fRef = 4*tanh(3*sin(4*t));  %Simple sine curve for now

end