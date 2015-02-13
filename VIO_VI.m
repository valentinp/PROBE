% demonstrates stereo visual odometry on an image sequence
disp('===========================');
addpath('vo');
addpath('utils');
addpath('~/Dropbox/PhD/Code/MATLAB/matlab_rosbag-0.4-mac64/');
rosBagFileName = '~/Desktop/Pioneer-VI/2015-01-28-16-11-17_1loop.bag';
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
first_frame = 2;
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
optParams.RANSACCostThresh = 2;
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
for frame=first_frame+1:1:last_frame
  
  % 1-index
  
  % read current images
   I1 = uint8(reshape(bagImageLeftVIData{frame}.data, viImageSize(1), viImageSize(2))');
   I2 = uint8(reshape(bagImageRightVIData{frame}.data, viImageSize(1), viImageSize(2))');

     
   I1 = I1(1:478, :);
   I2 = I2(1:478, :);
 
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
   C_21_est = T_camimu(1:3,1:3)*C_21_imu_est*T_camimu(1:3,1:3)'

    
  %Show image
  axes(ha1); cla;
  imagesc(I1);
  axis off; colormap(gray);
  
    matcherMex('push',I1,I2); 
    % match images
    matcherMex('match',2);
    p_matched = matcherMex('get_matches',2);
    % show matching results
    disp(['Number of matched points: ' num2str(length(p_matched))]);

    % showMatchedFeatures(I1, I2, p_matched(1:2,:)', p_matched(3:4,:)');
    % drawnow;

         %Triangulate points and prune any at Infinity
    [p_f1_1, p_f2_2] = triangulateAllPointsDirect(p_matched, calibParams);
    pruneId = isinf(p_f1_1(1,:)) | isinf(p_f1_1(2,:)) | isinf(p_f1_1(3,:)) | isinf(p_f2_2(1,:)) | isinf(p_f2_2(2,:)) | isinf(p_f2_2(3,:));
    p_f1_1 = p_f1_1(:, ~pruneId);
    p_f2_2 = p_f2_2(:, ~pruneId);
    
    %Select a random subset of 100
    selectIdx = randperm(size(p_f1_1,2),100);
    p_f1_1 = p_f1_1(:, selectIdx);
    p_f2_2 = p_f2_2(:, selectIdx);

    %Find inliers based on rotation matrix from IMU
    [p_f1_1, p_f2_2] = findInliersRot(p_f1_1, p_f2_2, C_21_est, optParams);
 fprintf('Tracking %d features.', size(p_f1_1,2));
    
    %Calculate initial guess using scalar weights, then use matrix weighted
    %non linear optimization
    R_1 = repmat(R, [1 1 size(p_f1_1, 2)]);
    R_2 = R_1;
    T_21_est = scalarWeightedPointCloudAlignment(p_f1_1, p_f2_2, C_21_est);
    T_21_opt = matrixWeightedPointCloudAlignment(p_f1_1, p_f2_2, R_1, R_2, T_21_est, calibParams, optParams);

        T_wcam = T_wcam*inv(T_21_opt);
        % update trajectory
         axes(ha2);
        plot(T_wcam(1,4),T_wcam(3,4),'g*');
         hold on;
         grid on;
         drawnow();
        k = k + 1
end

% close matcher
matcherMex('close');


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


