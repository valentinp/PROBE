clear all
clc
addpath('../utils');
addpath('../');
addpath('simulation');

y_var = 4*ones(4,1);
y_std = sqrt(y_var);
%Simulation
numPts = 100;


%Estimation
cu = 376.9974;
cv = 258.3066;
fu = 440.1944;
fv = 440.1944;
b = 0.5;

calibParams.c_u = cu;
calibParams.c_v = cv;
calibParams.f_u = fu;
calibParams.f_v = fv;
calibParams.b = b;

% Generate the measurments
%Generate the trajectory motion
close all;
%Simulation parameters
simSetup.imuRate = 10; % Hz
simSetup.cameraRate = 10; % Hz
simSetup.simTime =  20;    % seconds
simSetup.gyroNoiseStd = 0.1; 
simSetup.accelNoiseStd = 0;
simSetup.gyroBiasStd = 0.1; 
simSetup.accelBiasStd = 0;

T_camimu = eye(4);
[T_wImu_GT, imuData] = genTrajectoryCircle(simSetup);

T_wCam_GT = T_wImu_GT;
r_i_vk_i = NaN(3, size(T_wImu_GT, 3));
for i = 1:size(T_wImu_GT, 3)
    T_wCam_GT(:,:,i) = T_wImu_GT(:,:,i)*inv(T_camimu);
    T_0Imu = inv(T_wImu_GT(:,:,1))*T_wImu_GT(:,:,i);
    r_i_vk_i(:, i) = T_0Imu(1:3,4);
end

%Generate the true landmarks
landmarks_w = genLandmarks([-10,10], [-10,10],[0,5], numPts);

%%

M = [fu 0 cu 0;
    0 fv cv 0;
    fu 0 cu -fu*b;
    0 fv cv 0];

y_k_j = NaN(4,size(T_wCam_GT,3), numPts);
for step_i = 1:size(T_wCam_GT,3)
    step_i
    T_wc = T_wCam_GT(:,:,step_i);
    T_cw = inv(T_wc);
    for f_i = 1:numPts
        point = homo2cart(T_cw*cart2homo(landmarks_w(:, f_i)));
        pix = (1/point(3))*M*cart2homo(point)+ y_std'*randn(4, 1);
        if pix(1) > 0 && pix(1) < 1200 && pix(2) > 0 && pix(2) < 480
            y_k_j(:,step_i, f_i) = (1/point(3))*M*cart2homo(point)+ y_std'*randn(4, 1);
        else
            y_k_j(:,step_i, f_i) = -1*ones(4,1);
        end
    end
end

%% Export variables
w_vk_vk_i = imuData.measOmega;
v_vk_vk_i = imuData.measVel;

C_c_v = eye(3);
rho_v_c_v = zeros(3,1);

fileName = 'simData.mat';

size(w_vk_vk_i)
size(v_vk_vk_i)
size(y_k_j)

t = imuData.timestamps;
save(fileName, 'w_vk_vk_i','v_vk_vk_i','r_i_vk_i', 'cu','cv','fu','fv','b', 'y_k_j', 'C_c_v', 'rho_v_c_v', 't');


%   [p_f1_1, ~] = triangulateAllPointsNoCheck(y_1, calibParams);
%     [p_f2_2, ~] = triangulateAllPointsNoCheck(y_2, calibParams);
% 
%     [p_f1_1, p_f2_2, T_21_cam_est] = findInliersRANSAC(p_f1_1, p_f2_2);
%     R_1 = repmat(R, [1 1 size(p_f1_1, 2)]);
%     R_2 = R_1;
%     %T_21_cam_est = eye(4);
%     T_21_opt = matrixWeightedPointCloudAlignment(p_f1_1, p_f2_2, R_1, R_2, T_21_cam_est, calibParams);

