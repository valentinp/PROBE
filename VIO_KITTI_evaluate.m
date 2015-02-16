disp('===========================');
addpath('libviso2');
addpath('extraction/utils');
addpath('extraction/utils/devkit');
addpath('utils');
dataBaseDir = '/Users/valentinp/Desktop/KITTI/2011_09_26/2011_09_26_drive_0005_sync';
dataCalibDir = '/Users/valentinp/Desktop/KITTI/2011_09_26';

%% Get ground truth and import data
frameRange = 1:150;
%Image data
leftImageData = loadImageData([dataBaseDir '/image_00'], frameRange);
rightImageData = loadImageData([dataBaseDir '/image_01'], frameRange);
%%
%IMU data
[imuData, imuFrames] = loadImuData(dataBaseDir, leftImageData.timestamps);
%Ground Truth
T_wIMU_GT = getGroundTruth(dataBaseDir, imuFrames);
skipFrames = 1;

%% Load calibration and parameters
[T_camvelo_struct, P_rect_cam1] = loadCalibration(dataCalibDir);
T_camvelo = T_camvelo_struct{1}; 
T_veloimu = loadCalibrationRigid(fullfile(dataCalibDir,'calib_imu_to_velo.txt'));
T_camimu = T_camvelo*T_veloimu;

%Add camera ground truth
T_wCam_GT = T_wIMU_GT;

for i = 1:size(T_wIMU_GT, 3)
    T_wCam_GT(:,:,i) = T_wIMU_GT(:,:,i)*inv(T_camimu);
end

K= P_rect_cam1(:,1:3);
b_pix = P_rect_cam1(1,4);

cu = K(1,3);
cv = K(2,3);
fu = K(1,1);
fv = K(2,2);
b = -b_pix/fu; %The KITTI calibration supplies the baseline in units of pixels

calibParams.c_u = cu;
calibParams.c_v = cv;
calibParams.f_u = fu;
calibParams.f_v = fv;
calibParams.b = b;

%LIBVISO2 matching
param.f     = fu;
param.cu    = cu;
param.cv    = cv;
param.base  = b;
param.nms_n                  = 2;   % non-max-suppression: min. distance between maxima (in pixels)
param.nms_tau                = 50;  % non-max-suppression: interest point peakiness threshold
param.match_binsize          = 50;  % matching bin width/height (affects efficiency only)
param.match_radius           = 200; % matching radius (du/dv in pixels)
param.match_disp_tolerance   = 1;   % du tolerance for stereo matches (in pixels)
param.outlier_disp_tolerance = 5;   % outlier removal: disparity tolerance (in pixels)
param.outlier_flow_tolerance = 5;   % outlier removal: flow tolerance (in pixels)
param.multi_stage            = 1;   % 0=disabled,1=multistage matching (denser and faster)
param.half_resolution        = 1;   % 0=disabled,1=match at half resolution, refine at full resolution
param.refinement             = 0;   % refinement (0=none,1=pixel,2=subpixel)

%% Setup
addpath('settings');
addpath('utils');
addpath('learning');
useWeights = false;

R = diag(16*ones(4,1));
optParams.RANSACCostThresh = 3;
optParams.maxGNIter = 10;
optParams.lineLambda = 0.75;
optParams.LMlambda = 1e-5;


%% Load model
load('learnedProbeModels/2011_09_26_drive_0005_sync_learnedPredSpace.mat');
searchObject = KDTreeSearcher(learnedPredSpace.predVectors');
refWeight = mean(learnedPredSpace.weights);

%% Main loop

% create figure
figure('Color',[1 1 1]);
ha1 = axes('Position',[0.05,0.7,0.9,0.25]);
%axis off;
ha2 = axes('Position',[0.05,0.05,0.9,0.6]);
axis equal, grid on, hold on;

rng(42);
% init matcher
matcherMex('init',param);
% push back first images
I1 = uint8(leftImageData.rectImages(:,:,1));
I2 = uint8(rightImageData.rectImages(:,:,1));
matcherMex('push',I1,I2); 

numFrames = size(leftImageData.rectImages, 3);
k =1;
T_wcam = eye(4);
T_wcam_hist = T_wcam;

% history variables
firstState.C_vi = eye(3);
firstState.r_vi_i = zeros(3,1);
firstState.k = 1;
oldState = firstState;

for frame=2:skipFrames:numFrames
  
    %IMU Data
    imuMeasurement.omega = imuData.measOmega(:, frame-1);
    imuMeasurement.v = imuData.measVel(:, frame-1);
    deltaT = imuData.timestamps(frame) - imuData.timestamps(frame-1);
    
    %Get an estimate through IMU propagation
    newState = propagateState(oldState, imuMeasurement, deltaT);
    T_21_imu = getTransformation(oldState, newState);
    T_21_cam = T_camimu*T_21_imu*inv(T_camimu);
    oldState = newState;

    % read current images
    I1 = uint8(leftImageData.rectImages(:,:,frame));
    I2 = uint8(rightImageData.rectImages(:,:,frame));
 
 

    matcherMex('push',I1,I2); 
    % match images
    matcherMex('match',2);
    p_matched = matcherMex('get_matches',2);
    % show matching results
    %disp(['Number of matched points: ' num2str(length(p_matched))]);


      
      
    %Triangulate points and prune any at Infinity
    [p_f1_1, p_f2_2] = triangulateAllPointsDirect(p_matched, calibParams);
    pruneId =  p_matched(1,:) > 600 | isinf(p_f1_1(1,:)) | isinf(p_f1_1(2,:)) | isinf(p_f1_1(3,:)) | isinf(p_f2_2(1,:)) | isinf(p_f2_2(2,:)) | isinf(p_f2_2(3,:));
    p_f1_1 = p_f1_1(:, ~pruneId);
    p_f2_2 = p_f2_2(:, ~pruneId);
    
    %Select a random subset of 100
    selectIdx = randperm(size(p_f1_1,2), 25);
    p_f1_1 = p_f1_1(:, selectIdx);
    p_f2_2 = p_f2_2(:, selectIdx);
    p_matched = p_matched(:, selectIdx);
    
    
    %Find inliers based on rotation matrix from IMU
    %[p_f1_1, p_f2_2, T_21_est, inliers] = findInliersRot(p_f1_1, p_f2_2, T_21_cam(1:3,1:3), optParams);
    %[p_f1_1, p_f2_2, T_21_est, inliers] = findInliersRANSAC(p_f1_1, p_f2_2, optParams);
    
             %Plot image
%       axes(ha1); cla;
%       %imagesc(I1);
%       %hold on;
%       showMatchedFeatures(I1,I2,p_matched(5:6,inliers)', p_matched(7:8,inliers)'); 
%       axis off;
      

    inliers = 1:size(p_f1_1,2);
    %If desired find optimal weight for each observation.
    if useWeights
        [predVectors] = computePredVectors( p_matched(1:2,inliers), I1, [imuData.measAccel(:, frame-1); imuData.measOmega(:, frame-1)]);
        R_1 = NaN(4,4, length(inliers));
        predWeightList = NaN(1, length(inliers));
        for p_i = 1:length(inliers)
            predWeight = getPredVectorWeight(predVectors(:,p_i), searchObject, learnedPredSpace.weights, refWeight);
            R_1(:,:,p_i) = predWeight*R;
            predWeightList(p_i) = predWeight;
        end
        fprintf('mean pred weight: %.5f \n',mean(predWeight));
        R_2 = R_1;
    else
        R_1 = repmat(R, [1 1 size(p_f1_1, 2)]);
        R_2 = R_1;
    end
    T_21_est = scalarWeightedPointCloudAlignment(p_f1_1, p_f2_2,T_21_cam(1:3,1:3));
    T_21_opt = matrixWeightedPointCloudAlignment(p_f1_1, p_f2_2, R_1, R_2, T_21_est, calibParams, optParams);
    
    T_wcam = T_wcam*inv(T_21_opt);
    T_wcam_hist(:,:,end+1) = T_wcam;
    
    % update trajectory and plot
    axes(ha2);
    plot(T_wcam(1,4),T_wcam(3,4),'g*');
    hold on;
    grid on;
   drawnow();
    k = k + 1;
    fprintf('k: %d \n',k);
end

% close matcher
matcherMex('close');

p_vi_i = NaN(3, size(T_wCam_GT,3));
for j = frameRange
    T_wcam_gt =  inv(T_wCam_GT(:,:,1))*T_wCam_GT(:,:, j);
    p_vi_i(:,j) = T_wcam_gt(1:3,4);
end
translation = NaN(3, size(T_wcam_hist, 3));
for i = 1:size(T_wcam_hist, 3)
    T_wcam =  T_wcam_hist(:, :, i);
    translation(:,i) = T_wcam(1:3, 4);
end

%Plot error and variances
transErrVec = zeros(3, length(frameRange));
for i = frameRange
    transErrVec(:,i) = translation(:, i) - p_vi_i(:,i);
end
meanRMSE = mean(sqrt(sum(transErrVec.^2,1)/3))
finalErrorNorm = norm(transErrVec(:,end))

%% Create Search Tree
f = strsplit(dataBaseDir, '/');
f = strsplit(char(f(end)), '.');
fileName = char(f(1));
save(sprintf('plots/%s_noMore600Path.mat', fileName), 'translation');



%% IMU Only Integration
%Begin integration
xInit.p = zeros(3,1);
xInit.v = imuData.measVel(:,1);
xInit.b_g = zeros(3,1);
xInit.b_a = zeros(3,1);
xInit.q = [1; zeros(3,1)];
xState = xInit;
translation_imu = NaN(3, length(frameRange));
translation_imu(:,1) = zeros(3,1);
for imu_i = 2:length(frameRange)
    dt = imuData.timestamps(imu_i) - imuData.timestamps(imu_i - 1);
    [xState] = integrateIMU(xState, imuData.measAccel(:,imu_i), imuData.measOmega(:,imu_i), dt, zeros(3,1));
    T_wimu = [rotmat_from_quat(xState.q) xState.p; 0 0 0 1];
    T_wcam = T_camimu*T_wimu*inv(T_camimu);
    translation_imu(:,imu_i) = T_wcam(1:3,4);
end

%% Calculate statistics and plot
totalDist = 0;
p_vi_i = NaN(3, size(T_wCam_GT,3));
for j = frameRange
    if j > 1
        T_12 = inv(T_wCam_GT(:,:,j-1))*T_wCam_GT(:,:, j);
        totalDist = totalDist + norm(T_12(1:3,4));
    end
    T_wcam_gt =  inv(T_wCam_GT(:,:,1))*T_wCam_GT(:,:, j);
    p_vi_i(:,j) = T_wcam_gt(1:3,4);
end

f = strsplit(dataBaseDir, '/');
f = strsplit(char(f(end)), '.');
fileName = char(f(1));

translation = NaN(3, size(T_wcam_hist, 3));
for i = 1:size(T_wcam_hist, 3)
    T_wcam =  T_wcam_hist(:, :, i);
    translation(:,i) = T_wcam(1:3, 4);
end
% 
figure
plot(translation(1,:),translation(3,:), '-b');
 hold on;
plot(translation_imu(1,:),translation_imu(3,:), '-r');
plot(p_vi_i(1,frameRange),p_vi_i(3,frameRange), '-g');
title(sprintf('%s - Dist: %.5f m',fileName, totalDist), 'Interpreter', 'none');
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
legend('Optimized','IMU','Ground Truth');

%Plot error and variances
transErrVec = zeros(3, length(frameRange));
transErrVecIMU = zeros(3, length(frameRange));

for i = frameRange
    transErrVec(:,i) = translation(:, i) - p_vi_i(:,i);
    transErrVecIMU(:,i) = translation_imu(:, i) - p_vi_i(:,i);
end

meanRMSE = mean(sqrt(sum(transErrVec.^2,1)/3));
meanRMSEIMU = mean(sqrt(sum(transErrVecIMU.^2,1)/3));


figure
subplot(3,1,1)
plot(transErrVec(1,:), 'LineWidth', 1.2)
hold on
plot(transErrVecIMU(1,:), 'LineWidth', 1.2)
title(sprintf('Translational Error | Mean RMSE (Opt/IMU): %.5f/%.5f', meanRMSE, meanRMSEIMU))
legend('Opt', 'IMU Only')
grid on
ylabel('\delta r_x')
subplot(3,1,2)
plot(transErrVec(2,:), 'LineWidth', 1.2)
hold on
plot(transErrVecIMU(2,:), 'LineWidth', 1.2)
ylabel('\delta r_y')
grid on

subplot(3,1,3)
plot(transErrVec(3,:), 'LineWidth', 1.2)
hold on
plot(transErrVecIMU(3,:), 'LineWidth', 1.2)
ylabel('\delta r_z')
xlabel('t_k')

grid on

