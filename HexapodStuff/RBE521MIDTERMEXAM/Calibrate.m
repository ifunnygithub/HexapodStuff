clc;
clear all;

% Identify the real kinematic parameters given the nominal kinematic
% parameters and the ability to peturb and measure the system to solve a
% nonlinear least squares optimization problem using the cost function
% specified in CF.m

% Important: This is only used in the
% measurement phase! The rest of the program has no idea what the actual
% kinematic parameters are
% Real Kinematic Parameters
sReal = [96.6610, 22.2476, -122.4519, -120.6859, 24.7769, 91.3462;
    81.7602, 125.2511, 36.6453, -34.4565, -125.0489, -80.9866;
    1.0684, -0.5530,   4.3547,  -4.9014,  -4.8473,   0.2515];

uReal = [305.2599, -55.2814, -244.7954, -252.5755, -53.9678, 302.4266;
    115.0695, 322.9819, 208.0087, -211.8783, -320.6115, -109.4351;
    2.6210,   4.2181,   3.9365,   -3.0128,    4.3181,   3.3812];

LMinReal = [604.4299;
    607.2473;
    600.4441;
    605.9031;
    604.5251;
    600.0616];

realKinematicParameters = [sReal(:,1)', uReal(:,1)', LMinReal(1,1);
    sReal(:,2)', uReal(:,2)', LMinReal(2,1);
    sReal(:,3)', uReal(:,3)', LMinReal(3,1);
    sReal(:,4)', uReal(:,4)', LMinReal(4,1);
    sReal(:,5)', uReal(:,5)', LMinReal(5,1);
    sReal(:,6)', uReal(:,6)', LMinReal(6,1)];

% Nominal Kinematic Parameters
s = [92.1597, 27.055, -119.2146, -119.2146, 27.055,   92.1597;
    84.4488, 122.037, 37.5882,  -37.5882, -122.037, -84.4488;
    0,       0,       0,         0,        0,        0];

u = [305.4001, -56.4357, -248.9644,  -248.9644, -56.4357,   305.4001;
    111.1565,  320.0625, 208.9060,  -208.9060, -320.0625, -111.1565;
    0,         0,        0,          0,         0,         0];

LMinNominal = [604.8652;
    604.8652;
    604.8652;
    604.8652;
    604.8652;
    604.8652];

nominalKinematicParameters = [s(:,1)', u(:,1)', LMinNominal(1,1);
    s(:,2)', u(:,2)', LMinNominal(2,1);
    s(:,3)', u(:,3)', LMinNominal(3,1);
    s(:,4)', u(:,4)', LMinNominal(4,1);
    s(:,5)', u(:,5)', LMinNominal(5,1);
    s(:,6)', u(:,6)', LMinNominal(6,1)];

options = optimoptions('lsqnonlin');
options.Display = 'iter-detailed';
options.Algorithm = 'levenberg-marquardt';
% options.FiniteDifferenceType = 'central';
[identifiedKinematicParameters,resnorm,residual,exitflag,output] = lsqnonlin(@CF, nominalKinematicParameters,[],[],options);

% disp(nominalKinematicParameters);
% disp(realKinematicParameters);
% disp(identifiedKinematicParameters);

errorBeforeCalibration = zeros(42,1);
errorAfterCalibration = zeros(42,1);

for i = 0 : 5
    for j = 1 : 7
        % eBC(i+1,j) = norm(realKinematicParameters(i+1, j) - nominalKinematicParameters(i+1,j),2);
        % eAC(i+1,j) = norm(realKinematicParameters(i+1, j) - identifiedKinematicParameters(i+1,j),2);
        errorBeforeCalibration(i*7 + j) =  norm(realKinematicParameters(i+1, j) - nominalKinematicParameters(i+1,j),2);
        errorAfterCalibration(i*7 + j) =  norm(realKinematicParameters(i+1, j) - identifiedKinematicParameters(i+1,j),2);
    end
end
% disp(eBC);
% disp(eAC);
% disp(errorBeforeCalibration);
% disp(errorAfterCalibration);
calibrationData = [errorAfterCalibration, errorBeforeCalibration];
bar3(calibrationData);
ylabel('Kinematic Parameters')
zlabel('Error (mm)')
title('Kinematic Parameter Error')
legend('After Calibration', 'Before Calibration')
set(gca,'ytick',1:7:42,'yticklabel',compose('Leg %d',1:6))

% Note that unlike in the original paper, the bars in the bar graph are
% grouped by leg for easier readability.

% Something that immediately stands out in this graph is that the initial
% leg lengths before and after calibration are the same. After spending a
% few hours searching for the problem, I have concluded that lsqnonlin has
% most likely simply found a local minimum at the nominal leg lengths, as 
% it varies very slightly between iterations.