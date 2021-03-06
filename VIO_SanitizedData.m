% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear all;
clc;
addpath('utils');
addpath('testing');

data = load('extraction/2015-01-28-16-11-17_1loop.mat');
%Set number of landmarks
rng('shuffle');
numLandmarks = size(data.y_k_j,3);

%Set up appropriate structs
calibParams.c_u = data.cu;
calibParams.c_v = data.cv;
calibParams.f_u = data.fu;
calibParams.f_v = data.fv;
calibParams.b = data.b;

vehicleCamTransform.C_cv = data.C_c_v;
vehicleCamTransform.rho_cv_v = data.rho_v_c_v;
T_cv = [data.C_c_v -data.C_c_v*data.rho_v_c_v; 0 0 0 1];



%% Setup
addpath('settings');
%settings_dataset3
%settings_KITTI;
settings_viSensor;

%% Main Loop

firstState.k = kStart;
oldState = firstState;

%Triangulate all landmarks in first state
[oldPoints, oldPointIds] = triangulateAllPoints(data.y_k_j(:, kStart, :), calibParams);


%History
T_k0 = [firstState.C_vi -firstState.C_vi*firstState.r_vi_i; 0 0 0 1];
T_k0_imu = T_k0;
T_k0_hist = [];
T_k0_imu_hist = [];
T_k0_hist(:,:,1) = T_k0;
T_k0_imu_hist(:,:,1) = T_k0;

figure
for k=(kStart+1):kEnd
    
    fprintf('Processing %d \n', k);
    
    %IMU Data
    imuMeasurement.omega = data.w_vk_vk_i(:, k-1);
    imuMeasurement.v = data.v_vk_vk_i(:, k-1);
    deltaT = data.t(k) - data.t(k-1);
    
    %Get an estimate through IMU propagation
    newState = propagateState(oldState, imuMeasurement, deltaT);
    T_21_imu = getTransformation(oldState, newState);
    T_21_cam = T_cv*T_21_imu*inv(T_cv);
    
    %Perform frame-2-frame point cloud matching
    [newPoints, newPointIds] = triangulateAllPoints(data.y_k_j(:, k, :), calibParams);
    [p_f1_1, p_f2_2] = matchPointsBasedOnIds(oldPoints, oldPointIds, newPoints, newPointIds);
    
    
    
    

    %0-pt inlier check with IMU propagation
    [p_f1_1, p_f2_2] = findInliers(p_f1_1, p_f2_2, T_21_cam, optParams);
     
    
    %If there are enough points
    if size(p_f1_1, 2) > optParams.minFeaturesForOpt
%      plot3(p_f1_1(1,:), p_f1_1(2,:), p_f1_1(3,:), 'r*');
%      hold on;
%      plot3(p_f2_2(1,:), p_f2_2(2,:), p_f2_2(3,:), 'g*');
%      plot3([p_f1_1(1,:); p_f2_2(1,:)], [p_f1_1(2,:); p_f2_2(2,:)], [p_f1_1(3,:); p_f2_2(3,:)]);
%      pause();
% 
%     T_test =  inv(T_k0_hist(:, :, end));
%     T_test_imu =  inv(T_k0_imu_hist(:, :, end));
%     
%     plot3(T_test(1,4),T_test(2,4),T_test(3,4),'g*');
%     plot3(T_test_imu(1,4),T_test_imu(2,4),T_test_imu(3,4),'b*');
%     hold on;
%     grid on;
%     drawnow();

    
       %Perform RANSAC scalar weighted calculation
       [p_f1_1, p_f2_2, T_21_cam_est] = findInliersRANSAC(p_f1_1, p_f2_2,optParams);
        

        
        %Use matrix weighted approach to do final optimization`
        R_1 = repmat(R, [1 1 size(p_f1_1, 2)]);
        R_2 = R_1;
%         
%         R_2_jk = NaN(3,3,size(p_f2_2, 2));
%         R_1_jk = NaN(3,3,size(p_f1_1, 2));
%         for j=1:size(p_f1_1, 2)
%         R_2_jk(:,:,j) = dgdy(p_f2_2(:,j), calibParams)*R_2(:,:,j)*dgdy(p_f2_2(:,j), calibParams)';
%         R_1_jk(:,:,j) = dgdy(p_f1_1(:,j), calibParams)*R_1(:,:,j)*dgdy(p_f1_1(:,j), calibParams)';
%         end    
%         
%         [T_21_opt, resid] = abs_orient_points_ils(p_f1_1, p_f2_2, R_1_jk, R_2_jk, 10, T_21_cam(1:3,1:3));
%         
        %Hif
        
%         T_21_opt = [T_21_opt(1:3,1:3) -T_21_opt(1:3,1:3)*T_21_opt(1:3,4); 0 0 0 1];
        
        T_21_opt = matrixWeightedPointCloudAlignment(p_f1_1, p_f2_2, R_1, R_2, T_21_cam_est, calibParams, optParams);
        %T_21_opt = T_21_cam_est;
    else
        T_21_opt = T_21_cam;
    end
    
    %Transform back into IMU frame
    T_21_opt = inv(T_cv)*T_21_opt*T_cv;
    
    %Update old states
    oldState = newState;
    oldPoints = newPoints;
    oldPointIds = newPointIds;
    
    %Update history
    T_k0 = T_21_opt*T_k0;
    T_k0_imu = T_21_imu*T_k0_imu;
    
    T_k0_imu_hist(:,:,end+1) = T_k0_imu;
    T_k0_hist(:,:, end+1) = T_k0;
end



%% Plot
%Extract translations
translation = NaN(3, size(T_k0_hist, 3));
translation_imu = NaN(3, size(T_k0_hist, 3));
for i = 1:size(T_k0_hist, 3)
    T_0k =  inv(T_k0_hist(:, :, i));
    T_0k_imu = inv(T_k0_imu_hist(:,:,i));
    translation(:,i) = T_0k(1:3, 4);
    translation_imu(:,i) = T_0k_imu(1:3,4);
end
% 
plot3(translation(1,:),translation(2,:),translation(3,:), '-b');
 hold on;
plot3(translation_imu(1,:),translation_imu(2,:),translation_imu(3,:), '-r');
plot3(data.r_i_vk_i(1,kStart:kEnd),data.r_i_vk_i(2,kStart:kEnd),data.r_i_vk_i(3,kStart:kEnd), '-g');

xlabel('x');
ylabel('y');
zlabel('z');
grid on;
legend('Optimized', 'IMU', 'Ground Truth');

%%

%Plot error and variances
transErrVec = zeros(3, size(T_k0_hist,3));
transErrVecIMU = zeros(3, size(T_k0_hist,3));

for i = 1:size(T_k0_hist,3)
    transErrVec(:,i) = translation(:, i) - data.r_i_vk_i(:,kStart +i -1);
    transErrVecIMU(:,i) = translation_imu(:, i) - data.r_i_vk_i(:,kStart +i -1);
end

meanRMSE = mean(sqrt(sum(transErrVec.^2,1)/3));
meanRMSEIMU = mean(sqrt(sum(transErrVecIMU.^2,1)/3));

k1 = kStart;
k2 = kEnd;
figure
subplot(3,1,1)
plot(data.t(k1:k2), transErrVec(1,:), 'LineWidth', 1.2)
hold on
plot(data.t(k1:k2), transErrVecIMU(1,:), 'LineWidth', 1.2)
title(sprintf('Translational Error | Mean RMSE (Opt/IMU): %.5f/%.5f', meanRMSE,meanRMSEIMU))
legend('Opt', 'IMU');
ylabel('\delta r_x')


subplot(3,1,2)
plot(data.t(k1:k2), transErrVec(2,:), 'LineWidth', 1.2)
hold on
plot(data.t(k1:k2), transErrVecIMU(2,:), 'LineWidth', 1.2)
ylabel('\delta r_y')

subplot(3,1,3)
plot(data.t(k1:k2), transErrVec(3,:), 'LineWidth', 1.2)
hold on
plot(data.t(k1:k2), transErrVecIMU(3,:), 'LineWidth', 1.2)
ylabel('\delta r_z')
xlabel('t_k')
%set(gca,'FontSize',12)
%set(findall(gcf,'type','text'),'FontSize',12)
