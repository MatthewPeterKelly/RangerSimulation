function[uRef, kp, kd, qRef, dqRef, fsmMode] = walkControl(t, est, current, config)
% [uRef, kp, kd, qRef, dqRef, fsmMode] = walkControl(t, est, current, config)
%
% This function is a high-level controller. The robot should do a simple
% walking gait.
%

persistent currentIntegral
if isempty(currentIntegral)
    currentIntegral = [0;0;0];
end

dyn = config.dyn;
Phi = dyn.Phi;   %Ankle joint orientation constant
legLen = dyn.l;

% Unpack relative angles
qr = est(1);
qh = est(2);
% q0 = est(3);
% q1 = est(4);
dqr = est(5);
dqh = est(6);
% dq0 = est(7);
% dq1 = est(8);

%%% Get absolute angles:
[absAng, absRate] = getAngAbs(est(1:4), est(5:8), dyn.Phi);
% phi0 = absAng(1);
% phi1 = absAng(2);
th0 = absAng(3);
th1 = absAng(4);
% dphi0 = absRate(1);
% dphi1 = absRate(2);
dth0 = absRate(3);
dth1 = absRate(4);

%%% Integrate current integrals:
currentIntegral = currentIntegral + dyn.dt*current;

%%%% Figure out the contact mode
outerContact = est(9) >= 0.5;  % Outer contact active
innerContact = est(10) >= 0.5; % Inner contact active
if outerContact && innerContact
    contact = 'DS';
elseif outerContact && ~innerContact
    contact = 'S0';
elseif ~outerContact && innerContact
    contact = 'S1';
else
    contact = 'FL';
end

idx0 = 1;  %outer ankles
idx1 = 2;  %inner ankles
idxH = 3;  %hip

%%%% Initialize memory:
uRef = zeros(3,1);
kp = zeros(3,1);
kd = zeros(3,1);
qRef = zeros(3,1);
dqRef = zeros(3,1);

%%%% unpack the controller gains and parameters:  (decision variables)
W = config.walk;
targetStepLength = W.crit_step_length;

% Default values for the hip:
uRef(idxH) = hipGravityCompensation(th0, th1, contact, dyn);
kp(idxH) = W.hip_kp;
kd(idxH) = W.hip_kd;

%%%% Run controller:

FLIGHT = 0;
OUT_SWING = 1;
OUT_PUSH = 2;
OUT_DS = 3;
INN_SWING = 4;
INN_PUSH = 5;
INN_DS = 6;

[fsmMode, relHeight, currentIntegral] = runFiniteStateMachine(t,targetStepLength, W.doubleStance_duration,...
    th0, th1, dth0, dth1, innerContact, outerContact, legLen, currentIntegral);

% Target hip angle during flip-down  (Swing ankle should track straight
% down, holding desired step length, compensating for height)).
targetHipAngle = acos(1 - ( ( targetStepLength^2 + relHeight^2 )/( 2*legLen^2 ) ));

% Force DS mode to match PUSH mode. KISS principle
ctrlMode = fsmMode;
if ctrlMode == INN_DS
    ctrlMode = INN_PUSH;
elseif ctrlMode == OUT_DS
    ctrlMode = OUT_PUSH;
end

pushTarget = W.ank_push*W.ank_pushTarget + (1.0 - W.ank_push)*W.ank_holdLevel;
switch ctrlMode
    case OUT_SWING
        [qRef(idx0), dqRef(idx0)] = OuterAnkleTrackAbs(qr,dqr,W.ank_holdLevel,0.0,Phi);
        kp(idx0) = W.ank_stance_kp;
        kd(idx0) = W.ank_stance_kd;
        
        qRef(idx1) = W.ank_flipTarget;    %Flip-Up - track relative angle
        dqRef(idx1) = 0.0;
        kp(idx1) = W.ank_swing_kp;
        kd(idx1) = W.ank_swing_kd;
        
        [qRef(idxH), dqRef(idxH)] = HipScissorTrackOuter(th0, dth0, W.scissor_offset,W.scissor_gain);
        
    case OUT_PUSH
        [qRef(idx0), dqRef(idx0)] = OuterAnkleTrackAbs(qr,dqr,pushTarget,0.0,Phi);
        kp(idx0) = W.ank_push_kp;
        kd(idx0) = W.ank_push_kd;
        
        [qRef(idx1), dqRef(idx1)] = InnerAnkleTrackAbs(qr,dqr,qh,dqh,W.ank_holdLevel,0.0,Phi);
        kp(idx1) = W.ank_swing_kp;
        kd(idx1) = W.ank_swing_kd;
        
        qRef(idxH) = targetHipAngle;
        dqRef(idxH)= 0.0;
        
    case INN_SWING
        
        qRef(idx0) = W.ank_flipTarget;    %Flip-Up - track relative angle
        dqRef(idx0) = 0.0;
        kp(idx0) = W.ank_swing_kp;
        kd(idx0) = W.ank_swing_kd;
        
        [qRef(idx1), dqRef(idx1)] = InnerAnkleTrackAbs(qr,dqr,qh,dqh,W.ank_holdLevel,0.0,Phi);
        kp(idx1) = W.ank_stance_kp;
        kd(idx1) = W.ank_stance_kd;
        
        [qRef(idxH), dqRef(idxH)] = HipScissorTrackInner(th1, dth1, W.scissor_offset, W.scissor_gain);
        
    case INN_PUSH
        [qRef(idx0), dqRef(idx0)] = OuterAnkleTrackAbs(qr,dqr,W.ank_holdLevel,0.0,Phi);
        kp(idx0) = W.ank_swing_kp;
        kd(idx0) = W.ank_swing_kd;
        
        [qRef(idx1), dqRef(idx1)] = InnerAnkleTrackAbs(qr,dqr,qh,dqh,pushTarget,0.0,Phi);
        kp(idx1) = W.ank_push_kp;
        kd(idx1) = W.ank_push_kd;
        
        qRef(idxH) = -targetHipAngle;
        dqRef(idxH)= 0.0;
        
    case FLIGHT
        [qRef(idx0), dqRef(idx0)] = OuterAnkleTrackAbs(qr,dqr,W.ank_holdLevel,0.0,Phi);
        kp(idx0) = W.ank_swing_kp;
        kd(idx0) = W.ank_swing_kd;
        [qRef(idx1), dqRef(idx1)] = InnerAnkleTrackAbs(qr,dqr,qh,dqh,W.ank_holdLevel,0.0,Phi);
        kp(idx1) = W.ank_swing_kp;
        kd(idx1) = W.ank_swing_kd;
end

end  %%%% END MAIN FUNCTION



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Helper Functions                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%



function [xRef, vRef] = OuterAnkleTrackAbs(qr, dqr, phi0_ref,dphi0_ref,Phi)

%
% This function computes the controller set-points to track the desired
% absolute angle and rate.
%
% phi0 = Phi - qr - q0;
%
% q0 = Phi - qr - phi0;
% dq0 = -dqr - dphi0;


xRef = Phi - qr - phi0_ref;
vRef = -dqr - dphi0_ref;

end



function [xRef, vRef] = InnerAnkleTrackAbs(qr, dqr, qh, dqh, phi1_ref,dphi1_ref,Phi)

%
% This function computes the controller set-points to track the desired
% absolute angle and rate.
%
% phi0 = Phi - qr - q0;
%
% q0 = Phi - qr + qh - phi0;
% dq0 = -dqr + dqh - dphi0;


xRef = Phi - qr + qh - phi1_ref;
vRef = -dqr + dqh - dphi1_ref;

end

function [xRef, vRef] = HipScissorTrackOuter(th0, dth0, offset, rate)
%
% Causes the inner legs to track a linear function of the outer legs:

th1Ref = offset - rate*th0;
dth1Ref = -rate*dth0;

xRef = th1Ref-th0;
vRef = dth1Ref-dth0;

end

function [xRef, vRef] = HipScissorTrackInner(th1, dth1, offset, rate)
%
% Causes the outer legs to track a linear function of the inner legs:
%
% C.ref = C.th0*th0 + C.th1*th1;

th0Ref = offset - rate*th1;

dth0Ref = -rate*dth1;

xRef = th1-th0Ref;
vRef = dth1-dth0Ref;

end


function u = hipGravityCompensation(th0, th1, contact, dyn)
% Compensate for gravity, assuming that the system is a static
% double pendulum. This solves for the instantaneous hip torque to
% hold the swing leg in place.

uScale = dyn.m*dyn.g*dyn.c; % mass*gravity*length

switch contact
    case 'S0'
        u = uScale*sin(th1);
    case 'S1'
        u = -uScale*sin(th0);
    otherwise
        u = 0;
end

end


function [fsmState, relH, currentIntegral] = runFiniteStateMachine(time, critStepLen, dsDuration,...
    th0, th1, dth0, dth1, innerContact, outerContact, legLen, currentIntegral)
% Runs a simple finite state machine, based on SimBiCon ideas

persistent LAST_STATE SWITCH_TIME;

FLIGHT = 0;
OUT_SWING = 1;
OUT_PUSH = 2;
OUT_DS = 3;
INN_SWING = 4;
INN_PUSH = 5;
INN_DS = 6;

% Current index:
OUT = 1;
INN = 2;
HIP = 3;

if isempty(LAST_STATE) || time == 0  %Initialization
    LAST_STATE = FLIGHT;
    SWITCH_TIME = time;
    currentIntegral = zeros(3,1);
end

fsmState = LAST_STATE;
relH =  0;

switch fsmState
    case FLIGHT
        if innerContact
            fsmState = INN_SWING; SWITCH_TIME = time;
        elseif outerContact
            fsmState = OUT_SWING; SWITCH_TIME = time;
        end
        currentIntegral = zeros(3,1);
        
    case OUT_SWING
        [ankleDist, relH] = getRelativeAngleJointPos(th0, th1, legLen);
        if ankleDist > critStepLen
            fsmState = OUT_PUSH; SWITCH_TIME = time;
            currentIntegral = zeros(3,1);
        end
    case OUT_PUSH
        [~, relH] = getRelativeAngleJointPos(th0, th1, legLen);
currentIntegral(OUT)
        if innerContact
            fsmState = OUT_DS; SWITCH_TIME = time;
        end
    case OUT_DS
        [~, relH] = getRelativeAngleJointPos(th0, th1, legLen);
%         if time - SWITCH_TIME > dsDuration
currentIntegral(OUT)
        if currentIntegral(OUT) > 0.6    %%%% HACK %%%%
            fsmState = INN_SWING; SWITCH_TIME = time;
            currentIntegral = zeros(3,1);
        end
        
    case INN_SWING
        [ankleDist, relH] = getRelativeAngleJointPos(th1, th0, legLen);
        if ankleDist > critStepLen
            fsmState = INN_PUSH; SWITCH_TIME = time;
            currentIntegral = zeros(3,1);
        end
    case INN_PUSH
        [~, relH] = getRelativeAngleJointPos(th1, th0, legLen);
currentIntegral(INN)
        if outerContact
            fsmState = INN_DS; SWITCH_TIME = time;
        end
    case INN_DS
        [~, relH] = getRelativeAngleJointPos(th1, th0, legLen);
%         if time - SWITCH_TIME > dsDuration
currentIntegral(INN)
        if currentIntegral(INN) > 0.6    %%%% HACK %%%%
            fsmState = OUT_SWING; SWITCH_TIME = time;
            currentIntegral = zeros(3,1);
        end
        
    otherwise
        error('Invalid FSM state');
end


LAST_STATE = fsmState;

end %% runFiniteStateMachine
