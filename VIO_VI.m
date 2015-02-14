% demonstrates stereo visual odometry on an image sequence
clear all;
close all;
disp('===========================');
addpath('vo');
addpath('utils');
addpath('learning');

addpath('~/Dropbox/PhD/Code/MATLAB/matlab_rosbag-0.4-mac64/');
rosBagFileName = '~/Desktop/Pioneer-VI/2015-02-13-18-29-39-fwbw.bag';
viRightCamTopic = '/right/image_rect';
viLeftCamTopic = '/left/image_rect';
viconTopic = '/vicon/Pioneer1/Base';
imuTopic = '/imu0';

bag = ros.Bag.load(rosBagFileName);
bag.info()
bagImageRightVIData = bag.readAll({viRightCamTopic});
bagImageLeftVIData = bag.readAll({viLeftCamTopic});
viLeftcalib = bag.readAll({'/cam0/camera_info'}); %right
viRightcalib = bag.readAll({'/cam1/camera_info'}); %left
bagViconData = bag.readAll({viconTopic});
bagImuData = bag.readAll({imuTopic});

viImageSize = [752 480];

data = load('datasets/2015-01-28-16-11-17_1loop.mat');
param.f     = data.fu;
param.cu    = data.cu;
param.cv    = data.cv;
param.base  = data.b;
first_frame = 1;
calibParams.c_u = data.cu;
calibParams.c_v = data.cv;
calibParams.f_u = data.fu;
calibParams.f_v = data.fv;
calibParams.b = data.b;
last_frame  = length(bagImageRightVIData);

p_ci_i = [0.0602131703928; -0.00145860604554; -0.0465617957282];
R_ic = rotmat_from_quat([0.999996930467 -5.44616859609e-05 0.00246256274859 -0.000268097157994]');
R_ci = R_ic';
T_camimu = [R_ci -R_ci*p_ci_i; 0 0 0 1];


%%
% demonstrates sparse scene flow (quad matching = 2 consecutive stereo pairs)
% matching parameters
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

R = diag(25*ones(4,1));
optParams.RANSACCostThresh = 0.5*(0.1)^2;
optParams.maxGNIter = 10;
optParams.lineLambda = 0.75;
optParams.LMlambda = 1e-5;
%% Extract IMU timestamps
%Subtract gravity
imuData = NaN(6, length(bagImuData));
imuDataTimeStamps = NaN(1, length(bagImuData));
for imu_i=1:length(bagImuData)
    imuDataTimeStamps(imu_i) = bagImuData{imu_i}.header.stamp.time;
    imuData(:, imu_i) = [bagImuData{imu_i}.linear_acceleration; bagImuData{imu_i}.angular_velocity];
end

%%
% create figure
figure('Color',[1 1 1]);
ha1 = axes('Position',[0.05,0.7,0.9,0.25]);
%axis off;
ha2 = axes('Position',[0.05,0.05,0.9,0.6]);
axis equal, grid on, hold on;
%%

% init matcher
matcherMex('init',param);

% push back first images
I1 = uint8(reshape(bagImageLeftVIData{first_frame}.data, viImageSize(1), viImageSize(2))');
I2 = uint8(reshape(bagImageRightVIData{first_frame}.data, viImageSize(1), viImageSize(2))');
I1 = I1(1:478, :);
I2 = I2(1:478, :);
matcherMex('push',I1,I2); 
previousViTime = bagImageLeftVIData{first_frame}.header.stamp.time;
xInit.p = zeros(3,1);
xInit.v = zeros(3,1);
xInit.b_g = zeros(3,1);
xInit.b_a = zeros(3,1);
xInit.q = [1; zeros(3,1)];
xState = xInit;

 
k =1;
T_wcam = eye(4);
T_wcam_hist = T_wcam;
load('cluster.mat');

for frame=first_frame+1:1:last_frame-1
  
  % 1-index
  
  % read current images
   I1 = uint8(reshape(bagImageLeftVIData{frame}.data, viImageSize(1), viImageSize(2))');
   I2 = uint8(reshape(bagImageRightVIData{frame}.data, viImageSize(1), viImageSize(2))');

     
   I1 = I1(1:478, :);
   I2 = I2(1:478, :);
   
   bm = blurMetric(I1);

 
   %IMU data
   currentViTime = bagImageLeftVIData{frame}.header.stamp.time;
   imuMask = imuDataTimeStamps >= previousViTime & imuDataTimeStamps < currentViTime;
   imuDataWindowTimeStamps = imuDataTimeStamps(imuMask);
   previousViTime = currentViTime;
   imuDataWindow = imuData(:, imuMask);
   xPrevState = xState; 
   for imusub_i = 2:size(imuDataWindow,2)
        dt = imuDataWindowTimeStamps(imusub_i) - imuDataWindowTimeStamps(imusub_i-1);
        [xState] = integrateIMU(xState, imuDataWindow(1:3, imusub_i-1), imuDataWindow(4:6, imusub_i-1), dt, zeros(3,1));
   end
   %xState.q = q_wimu
   C_21_imu_est = rotmat_from_quat(xState.q)'*rotmat_from_quat(xPrevState.q);
   C_21_est = T_camimu(1:3,1:3)*C_21_imu_est*T_camimu(1:3,1:3)';

    
  %Show image
  axes(ha1); cla;
  imagesc(I1);
  axis off; colormap(gray);
  
    matcherMex('push',I1,I2); 
    % match images
    matcherMex('match',2);
    p_matched = matcherMex('get_matches',2);

    % showMatchedFeatures(I1, I2, p_matched(1:2,:)', p_matched(3:4,:)');
    % drawnow;

         %Triangulate points and prune any at Infinity
    selectIdx = randperm(size(p_matched,2), 200);
    [p_f1_1, p_f2_2] = triangulateAllPointsDirect(p_matched(:,selectIdx), calibParams);
    
    pruneId = isinf(p_f1_1(1,:)) | isinf(p_f1_1(2,:)) | isinf(p_f1_1(3,:)) | isinf(p_f2_2(1,:)) | isinf(p_f2_2(2,:)) | isinf(p_f2_2(3,:));
    p_f1_1 = p_f1_1(:, ~pruneId);
    p_f2_2 = p_f2_2(:, ~pruneId);
 

    %Find inliers based on rotation matrix from IMU
    
    [p_f1_1, p_f2_2, T_21_est] = findInliersRot(p_f1_1, p_f2_2, C_21_est, optParams,calibParams);
    %T_21_cam_best = scalarWeightedPointCloudAlignment(p_f1_1, p_f2_2, C_21_est);
    fprintf('Tracking %d features. \n', size(p_f1_1,2));
    
    %Calculate initial guess using scalar weights, then use matrix weighted
    %non linear optimization
    if bm > 0.4
        R_pix = R;
        disp('Blurry brah!');
    else
        R_pix = R;
    end
    
    
    R_1 = repmat(R_pix, [1 1 size(p_f1_1, 2)]);
    R_2 = R_1;
    
    T_21_opt = matrixWeightedPointCloudAlignment(p_f1_1, p_f2_2, R_1, R_2, T_21_est, calibParams, optParams);
    
        
    T_wcam = T_wcam*inv(T_21_opt);
    T_wcam_hist(:,:,end+1) = T_wcam;
    
    % update trajectory
         axes(ha2);
        plot(T_wcam(1,4),T_wcam(3,4),'b*');
         hold on;
        %plot(p_camw_w_gt(1, frame-1),p_camw_w_gt(3, frame-1),'g*');
         grid on;
         drawnow();
        k = k + 1;
        fprintf('k:%d \n',k);
end

T_wcam(1:3,4)
norm(T_wcam(1:3,4))
% close matcher
matcherMex('close');

%% Load 'ground truth'
% load('1loopGT.mat');
% 
% %Calculate statistics and plot
% totalDist = 0;
% frameRange = 1:1:last_frame-2;
% for j = frameRange
%     if j > 1
%         totalDist = totalDist + norm(p_camw_w_gt(:,j) - p_camw_w_gt(:,j-1));
%     end
% end
% translation = NaN(3, size(T_wcam_hist, 3));
% for i = 1:size(T_wcam_hist, 3)
%     T_wcam =  T_wcam_hist(:, :, i);
%     translation(:,i) = T_wcam(1:3, 4);
% end
% %Plot error and variances
% transErrVec = zeros(3, length(frameRange));
% 
% for i = frameRange
%     transErrVec(:,i) = translation(:, i) - p_camw_w_gt(:,i);
% end
% 
% meanRMSE = mean(sqrt(sum(transErrVec.^2,1)/3));
% 
% 
% % 
% figure
% plot(translation(1,:),translation(3,:), '-b');
%  hold on;
% plot(p_camw_w_gt(1,frameRange),p_camw_w_gt(3,frameRange), '-g');
% title(sprintf('VI Sensor - ARMSE: %.5f, Dist: %.5f m', meanRMSE, totalDist), 'Interpreter', 'none');
% xlabel('x');
% ylabel('y');
% zlabel('z');
% grid on;
% legend('Optimized','Ground Truth');



%% Calculate Blur Metric
% addpath('learning');
% bmHist = [];
% for frame=first_frame+1:1:last_frame
%   
%   
%   % read current images
%    I1 = uint8(reshape(bagImageLeftVIData{frame}.data, viImageSize(1), viImageSize(2))');
%    I2 = uint8(reshape(bagImageRightVIData{frame}.data, viImageSize(1), viImageSize(2))');
%    I1 = I1(1:478, :);
%    I2 = I2(1:478, :);
%     bm = blurMetric(I1);
%     bmHist(end+1) = bm;
% end
% plot(bmHist);
%% Plot VICON Data if desired
% addpath('extraction/utils')
% figure
% T_p1_hist = [];
% for v_i = 1:length(bagViconData)
%     t = bagViconData{v_i}.transform.translation;
%     q = bagViconData{v_i}.transform.rotation;
%     if v_i == 0
%         T_g1 = [quatToRotMat(q)' t; 0 0 0 1]; 
%         T_p1_hist(:,:, v_i) = eye(4);
%     else
%         T_p1_hist(:,:, v_i) = [quatToRotMat(q)' t; 0 0 0 1];
%     end
% end
% 
% t_p1_hist = [];
% for v_i = 1:length(bagViconData)
%     t_p1_hist(:, v_i) = T_p1_hist(1:3, 4, v_i);
% end
% plot(t_p1_hist(1,:), t_p1_hist(2,:), '*')

%% Cluster data
addpath('learning');
bmHist = [];
allPredVectors = []
for frame=first_frame+1:1:last_frame-2
    frame
   % read current images
   I1 = (reshape(bagImageLeftVIData{frame}.data, viImageSize(1), viImageSize(2))');
   %I2 = (reshape(bagImageRightVIData{frame}.data, viImageSize(1), viImageSize(2))');
   I1 = I1(1:478, :);
   %I2 = I2(1:478, :);
    
    keyPoints = detectSURFFeatures(mat2gray(I1));
    keyPointPixels = keyPoints.Location(:,:)';
    currentViTime = bagImageLeftVIData{frame}.header.stamp.time;

    %Computes Prediction Space points based on the image and keypoint position
    imu_i = findClosestTimestamp(currentViTime, imuDataTimeStamps);
    imuDataRecord = imuData(:, imu_i);
    
    predVectors = computePredVectors(keyPointPixels, I1, imuDataRecord);
    allPredVectors = [allPredVectors predVectors];

end

%%
numClusters = 10;
Km = 1.5;
[idx, C,~,D] = kmeans(allPredVectors', numClusters);


% [COEFF,SCORE, latent] = princomp(allPredVectors');
% latent
% COEFF


%Determine the mean distance of points within a cluster to the cluster's centroid
%This sets boundaries

meanCentroidDists = zeros(numClusters, 1);
predVectorsWithinBoundaries = 0;
for ic = 1:numClusters
    meanCentroidDists(ic) = Km*mean(D(idx == ic, ic));
    predVectorsWithinBoundaries = predVectorsWithinBoundaries + sum(D(idx == ic, ic) < meanCentroidDists(ic));
end

inFraction = predVectorsWithinBoundaries/size(allPredVectors,2);

clusteringModel.clusterNum = numClusters;
clusteringModel.centroids = C';
clusteringModel.threshDists = meanCentroidDists;

save('cluster.mat', 'clusteringModel');



