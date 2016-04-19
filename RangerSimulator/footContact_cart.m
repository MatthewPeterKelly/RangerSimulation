function [x,y] = footContact_cart(footAngle)
% [x,y] = footContact_cart(footAngle)
%
% Given the angle of Ranger's foot, compute the position of the contact
% point with respect the the center of the ankle joint.
%
% x = horizontal position of the contact point, wrt ankle joint
% y = vertical position of the contact point, wrt ankle joint
%
%

footAngle = wrapToPi(footAngle);

qFoot = [-3.14159265, -3.09970475, -3.05781685, -3.01592895, -2.97404105, -2.93215314, -2.89026524, -2.84837734, -2.80648944, -2.76460154, -2.72271363, -2.68082573, -2.63893783, -2.59704993, -2.55516202, -2.51327412, -2.47138622, -2.42949832, -2.38761042, -2.34572251, -2.30383461, -2.26194671, -2.22005881, -2.17817091, -2.13628300, -2.09439510, -2.05250720, -2.01061930, -1.96873140, -1.92684349, -1.88495559, -1.84306769, -1.80117979, -1.75929189, -1.71740398, -1.67551608, -1.63362818, -1.59174028, -1.54985238, -1.50796447, -1.46607657, -1.42418867, -1.38230077, -1.34041287, -1.29852496, -1.25663706, -1.21474916, -1.17286126, -1.13097336, -1.08908545, -1.04719755, -1.00530965, -0.96342175, -0.92153385, -0.87964594, -0.83775804, -0.79587014, -0.75398224, -0.71209433, -0.67020643, -0.62831853, -0.58643063, -0.54454273, -0.50265482, -0.46076692, -0.41887902, -0.37699112, -0.33510322, -0.29321531, -0.25132741, -0.20943951, -0.16755161, -0.12566371, -0.08377580, -0.04188790, 0.00000000, 0.04188790, 0.08377580, 0.12566371, 0.16755161, 0.20943951, 0.25132741, 0.29321531, 0.33510322, 0.37699112, 0.41887902, 0.46076692, 0.50265482, 0.54454273, 0.58643063, 0.62831853, 0.67020643, 0.71209433, 0.75398224, 0.79587014, 0.83775804, 0.87964594, 0.92153385, 0.96342175, 1.00530965, 1.04719755, 1.08908545, 1.13097336, 1.17286126, 1.21474916, 1.25663706, 1.29852496, 1.34041287, 1.38230077, 1.42418867, 1.46607657, 1.50796447, 1.54985238, 1.59174028, 1.63362818, 1.67551608, 1.71740398, 1.75929189, 1.80117979, 1.84306769, 1.88495559, 1.92684349, 1.96873140, 2.01061930, 2.05250720, 2.09439510, 2.13628300, 2.17817091, 2.22005881, 2.26194671, 2.30383461, 2.34572251, 2.38761042, 2.42949832, 2.47138622, 2.51327412, 2.55516202, 2.59704993, 2.63893783, 2.68082573, 2.72271363, 2.76460154, 2.80648944, 2.84837734, 2.89026524, 2.93215314, 2.97404105, 3.01592895, 3.05781685, 3.09970475, 3.14159265];  % Foot orientation 
x = [-0.01230147, -0.01446038, -0.01608101, -0.01743493, -0.01861876, -0.01968684, -0.02066706, -0.02157224, -0.02241213, -0.02319748, -0.02393009, -0.02464545, -0.08564472, -0.08842242, -0.08956595, -0.08987055, -0.08956151, -0.08880657, -0.08765650, -0.08618160, -0.08441079, -0.08238131, -0.08012651, -0.07764719, -0.07498293, -0.07213293, -0.06910637, -0.06593796, -0.06261793, -0.05916274, -0.05558015, -0.05188572, -0.04808686, -0.04418364, -0.04019879, -0.03613243, -0.03199239, -0.02779364, -0.02354389, -0.01924401, -0.01490894, -0.01054664, -0.00616506, -0.00177223, 0.00261698, 0.00700140, 0.01136614, 0.01571006, 0.02001129, 0.02427573, 0.02848141, 0.03262741, 0.03669864, 0.04068714, 0.04457755, 0.04836175, 0.05202402, 0.05555612, 0.05892588, 0.06211596, 0.06508359, 0.06774983, 0.06990156, 0.06770084, 0.06224322, 0.05684427, 0.05144638, 0.04602497, 0.04051990, 0.03490448, 0.02916246, 0.02331627, 0.01745469, 0.01164590, 0.00589993, 0.00011625, -0.00584612, -0.01205016, -0.01100361, -0.00899065, -0.00684238, -0.00462221, -0.00235807, -0.00006218, 0.00224938, 0.00456831, 0.00689025, 0.00920692, 0.01150611, 0.01379135, 0.01605040, 0.01827895, 0.02046861, 0.02261502, 0.02471807, 0.02676519, 0.02875201, 0.03067425, 0.03252320, 0.03428990, 0.03597446, 0.03756309, 0.03905110, 0.04042382, 0.04167093, 0.04278181, 0.04374019, 0.04452361, 0.04509726, 0.04543137, 0.04544316, 0.04499784, 0.04356384, 0.00132433, 0.00104270, 0.00095861, 0.00094894, 0.00099350, 0.00108048, 0.00120142, 0.00134460, 0.00150978, 0.00168852, 0.00187896, 0.00207596, 0.00227928, 0.00248381, 0.00268935, 0.00289246, 0.00308976, 0.00327954, 0.00346169, 0.00363300, 0.00379031, 0.00393214, 0.00405553, 0.00415765, 0.00423579, 0.00428584, 0.00430241, 0.00428034, 0.00421178, 0.00408483, 0.00388670, 0.00359217, 0.00316416, 0.00253006, 0.00152058, -0.00046578, -0.00829830, -0.01230147];  % relative horizontal position 
y = [-0.02304208, -0.02360534, -0.02424623, -0.02494889, -0.02570448, -0.02650713, -0.02735258, -0.02823745, -0.02915888, -0.03011436, -0.03110159, -0.03211847, -0.03436125, -0.03801622, -0.04174774, -0.04550811, -0.04926794, -0.05300525, -0.05670231, -0.06034418, -0.06391792, -0.06741208, -0.07081637, -0.07412145, -0.07731876, -0.08040041, -0.08335908, -0.08618800, -0.08888086, -0.09143179, -0.09383537, -0.09608656, -0.09818071, -0.10011356, -0.10188121, -0.10348016, -0.10490724, -0.10615966, -0.10723501, -0.10813122, -0.10884659, -0.10937979, -0.10972985, -0.10989619, -0.10987857, -0.10967714, -0.10929240, -0.10872527, -0.10797700, -0.10704927, -0.10594413, -0.10466402, -0.10321182, -0.10159082, -0.09980477, -0.09785789, -0.09575497, -0.09350137, -0.09110319, -0.08856749, -0.08590266, -0.08311939, -0.08023372, -0.07731660, -0.07459524, -0.07210119, -0.06983314, -0.06779149, -0.06597853, -0.06439832, -0.06305607, -0.06195674, -0.06110291, -0.06049371, -0.06012632, -0.05999995, -0.06011911, -0.06049316, -0.06098875, -0.06140823, -0.06174017, -0.06198049, -0.06212680, -0.06217757, -0.06213184, -0.06198906, -0.06174903, -0.06141188, -0.06097801, -0.06044807, -0.05982300, -0.05910395, -0.05829234, -0.05738982, -0.05639827, -0.05531984, -0.05415690, -0.05291209, -0.05158829, -0.05018869, -0.04871674, -0.04717622, -0.04557126, -0.04390637, -0.04218650, -0.04041713, -0.03860436, -0.03675509, -0.03487726, -0.03298026, -0.03107575, -0.02917959, -0.02731893, -0.02643145, -0.02638363, -0.02634201, -0.02630228, -0.02626175, -0.02621841, -0.02617072, -0.02611748, -0.02605775, -0.02599082, -0.02591614, -0.02583332, -0.02574210, -0.02564232, -0.02553396, -0.02541705, -0.02529177, -0.02515833, -0.02501709, -0.02486845, -0.02471292, -0.02455112, -0.02438375, -0.02421164, -0.02403574, -0.02385716, -0.02367717, -0.02349728, -0.02331926, -0.02314529, -0.02297805, -0.02282102, -0.02267894, -0.02255875, -0.02247208, -0.02244399, -0.02259727, -0.02304208];  % relative verical position 

p = pwPoly2(qFoot,[x;y],footAngle);  %Piecewise quadratic interpolation

x = p(1,:);
y = p(2,:);

end