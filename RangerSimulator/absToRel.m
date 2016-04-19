function qRel = absToRel(qAbs)
% qRel = absToRel(qAbs)
% 
% This function converts from absolute angles to relative angles for the
% Ranger robot
%
% INPUTS:
%    qAbs = [qf1; qf2; ql1; ql2] = z(3:6);
%
% OUTPUTS:
%   qRel = [qHip; qAnkle1; qAnkle2]
%

qf1 = qAbs(1);
qf2 = qAbs(2);
ql1 = qAbs(3);
ql2 = qAbs(4);

qh = (ql2-ql1);   %Hip angle
qa1 = (qf1-ql1);   %ankle one angle
qa2 = (qf2-ql2);   %Ankle two angle

qRel = [qh;qa1;qa2];

end