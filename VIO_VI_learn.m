clear;
close all;
disp('===========================');
addpath('libviso2');
addpath('utils');
addpath('learning');
addpath('groundtruth/');


addpath('~/Dropbox/PhD/Code/MATLAB/matlab_rosbag-0.4-mac64/');
rosBagFileName = '/Users/valentinp/Desktop/Pioneer-VI/2015-02-18-12-43-18.bag';
gpsFile = '/Users/valentinp/Desktop/Pioneer-VI/GPS/rover3_rtk.mat';

%Set up topics
viRightCamTopic = '/right/image_rect';
viLeftCamTopic = '/left/image_rect';
imuTopic = '/imu0';

%Load the bag
bag = ros.Bag.load(rosBagFileName);
bag.info()
bagImageRightVIData = bag.readAll({viRightCamTopic});
bagImageLeftVIData = bag.readAll({viLeftCamTopic});

%right
viLeftcalib = bag.readAll({'/cam0/camera_info'}); 
%left
viRightcalib = bag.readAll({'/cam1/camera_info'}); 
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
last_frame  = length(bagImageRightVIData)-5;

%Set up IMU to Cam transform
p_ci_i = [0.0602131703928; -0.00145860604554; -0.0465617957282];
R_ic = rotmat_from_quat([0.999996930467 -5.44616859609e-05 0.00246256274859 -0.000268097157994]');
R_ci = R_ic';
T_camimu = [R_ci -R_ci*p_ci_i; 0 0 0 1];


%%
% demonstrates sparse scene flow (quad matching = 2 consecutive stereo pairs)
% matching parameters
param.nms_n                  = 3;   % non-max-suppression: min. distance between maxima (in pixels)
param.nms_tau                = 50;  % non-max-suppression: interest point peakiness threshold
param.match_binsize          = 50;  % matching bin width/height (affects efficiency only)
param.match_radius           = 200; % matching radius (du/dv in pixels)
param.match_disp_tolerance   = 2;   % du tolerance for stereo matches (in pixels)
param.outlier_disp_tolerance = 5;   % outlier removal: disparity tolerance (in pixels)
param.outlier_flow_tolerance = 5;   % outlier removal: flow tolerance (in pixels)
param.multi_stage            = 1;   % 0=disabled,1=multistage matching (denser and faster)
param.half_resolution        = 1;   % 0=disabled,1=match at half resolution, refine at full resolution
param.refinement             = 1;   % refinement (0=none,1=pixel,2=subpixel)

%% Setup
%% Setup
addpath('settings');
addpath('utils');
addpath('learning');

R = 25*eye(4);
optParams = {};
optParams.RANSACCostThresh = 1e-2;
optParams.OutlierThresh = 1e-3;
optParams.maxProbeWeight = 100;
trialType = 1;
switch trialType
    case 1
        useWeights = false;
        optParams.RANSACMaxIterations = round(log(1-0.99)/log(1-(1-0.5)^3));
        caseString = 'nominal';
    case 2
        useWeights = false;
        optParams.RANSACMaxIterations = round(log(1-0.9999)/log(1-(1-0.5)^3));
        caseString = 'aggressive';
    case 3
        useWeights = true;
        caseString = 'probe';
end
optParams.maxGNIter = 10;
optParams.lineLambda = 0.75;
optParams.LMlambda = 1e-5;
optParams
caseString
%% Extract IMU timestamps
imuData = NaN(6, length(bagImuData));
imuDataTimeStamps = NaN(1, length(bagImuData));
for imu_i=1:length(bagImuData)
    imuDataTimeStamps(imu_i) = bagImuData{imu_i}.header.stamp.time;
    imuData(:, imu_i) = [bagImuData{imu_i}.linear_acceleration; bagImuData{imu_i}.angular_velocity];
end
%% Extract GPS timestamps
% Calculate GPS Time
rtk_data = load(gpsFile);
gpsStartTime = rtk_data.initialTime;
gpsPointsNum = size(rtk_data.xyz,1);
timeComponents = strsplit(gpsStartTime, ':');
timeVec = [2015,2,18,str2num(timeComponents{1}),str2num(timeComponents{2}), str2num(timeComponents{3})];
gpsStartMatlabDateNum = datenum(timeVec);
datestr(gpsStartMatlabDateNum)
gpsStartUnixTime = double((gpsStartMatlabDateNum - 719529)*86400 + 5*3600);
gpsTimeStamps = double(1:gpsPointsNum) + gpsStartUnixTime;

%%
%create figure
figure('Color',[1 1 1]);
ha1 = axes('Position',[0.05,0.7,0.9,0.25]);

axis off;
ha2 = axes('Position',[0.05,0.05,0.9,0.6]);
axis equal, grid on, hold on;
%% Which frames do we want to use
frameList = first_frame+1:3:last_frame;
%% Extract camera times
camTimeStamps = NaN(1, length([1 frameList]));
k = 1;
for cam_i=[1 frameList]
    camTimeStamps(k) = bagImageLeftVIData{cam_i}.header.stamp.time;
    k = k + 1;
end

%%
repeatIter =  10;
meanRMSEHist = [];
learnedPredSpace = {};
learnedPredSpace.predVectors = [];
learnedPredSpace.weights = [];
p_wcam_hist = [];

for repeat_i = 1:repeatIter
rng('shuffle');
usedPredVectors = [];
% init matcher
matcherMex('init',param);

% push back first images
I1prev = uint8(reshape(bagImageLeftVIData{first_frame}.data, viImageSize(1), viImageSize(2))');
I2prev = uint8(reshape(bagImageRightVIData{first_frame}.data, viImageSize(1), viImageSize(2))');
I1prev = I1prev(1:478, :);
I2prev = I2prev(1:478, :);
matcherMex('push',I1prev,I2prev); 
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
p_wcam_hist(:, 1, repeat_i) = zeros(3,1);

for frame=frameList
  
  
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
   
   C_21_imu_est = rotmat_from_quat(xState.q)'*rotmat_from_quat(xPrevState.q);
   C_21_est = T_camimu(1:3,1:3)*C_21_imu_est*T_camimu(1:3,1:3)';

   
    matcherMex('push',I1,I2); 
    % match images
    matcherMex('match',2);
    p_matched = matcherMex('get_matches',2);

    
    %Triangulate points and prune any at Infinity
    numFts = 100;
         
    if size(p_matched,2) > numFts
        selectIdx = randperm(size(p_matched,2), numFts);
    else
        selectIdx = 1:size(p_matched,2);
    end
    p_matched = p_matched(:,selectIdx);
    [p_f1_1, p_f2_2] = triangulateAllPointsDirect(p_matched, calibParams);
       pruneId = isinf(p_f1_1(1,:)) | isinf(p_f1_1(2,:)) | isinf(p_f1_1(3,:)) | isinf(p_f2_2(1,:)) | isinf(p_f2_2(2,:)) | isinf(p_f2_2(3,:));
    p_f1_1 = p_f1_1(:, ~pruneId);
    p_f2_2 = p_f2_2(:, ~pruneId);
    
    %Select a subset
    %selectIdx = randperm(size(p_f1_1,2), 5);
    numLm = size(p_f1_1,2);
    skip = floor(numLm/repeatIter);
    buckets = [1:skip:numLm - mod(numLm, skip); skip:skip:numLm];
    
    
    selectIdx = buckets(1, repeat_i):buckets(2, repeat_i);
    rng(42); randOrder = randperm(numLm,numLm);
    selectIdx = randOrder(selectIdx);
    
    p_f1_1 = p_f1_1(:, selectIdx);
    p_f2_2 = p_f2_2(:, selectIdx);
    p_matched = p_matched(:, selectIdx);
    inliers = 1:size(p_f1_1,2);

    %Find inliers based on rotation matrix from IMU
     %[p_f1_1, p_f2_2, T_21_est, inliers] = findInliersRot(p_f1_1, p_f2_2, T_21_cam(1:3,1:3), optParams);
    [predVectors] = computePredVectors( p_matched(1:2,inliers), I1, I1prev, imuDataWindow(:, end));
    %usedPredVectors(:,end+1:end+size(predVectors, 2)) = predVectors;
    %fprintf('Tracking %d features.', size(p_f1_1,2));
    
    %Calculate initial guess using scalar weights, then use matrix weighted
    %non linear optimization

    R_1 = repmat(R, [1 1 size(p_f1_1, 2)]);
    R_2 = R_1;
    T_21_est = scalarWeightedPointCloudAlignment(p_f1_1, p_f2_2,C_21_est);
    T_21_opt = matrixWeightedPointCloudAlignment(p_f1_1, p_f2_2, R_1, R_2, T_21_est, calibParams, optParams);

    
    
    %Prune any vectors with 0 entries
    pruneWeightsIdx = predVectors(1,:) == 0 | predVectors(2,:) == 0 | predVectors(3,:) == 0 | predVectors(4,:) == 0 | predVectors(5,:) == 0 | predVectors(6,:) == 0 | predVectors(7,:) == 0;
    predVectors = predVectors(:,~pruneWeightsIdx);
    usedPredVectors(:,end+1:end+size(predVectors, 2)) = predVectors;

   
 
    T_wcam = T_wcam*inv(T_21_opt);
    T_wcam_hist(:,:,end+1) = T_wcam;
    
    p_wcam_hist(:, frame, repeat_i) = T_wcam(1:3,4);
    
    % update trajectory and plot
%     axes(ha2);
%     plot(T_wcam(1,4),T_wcam(3,4),'g*');
%     hold on;
%     grid on;
%    drawnow();
    I1prev = I1;
    I2prev = I2;
    k = k + 1;
    fprintf('k:%d, repeat_i: %d \n',k,repeat_i);
end

% close matcher
matcherMex('close');

%Calculate error
translation = NaN(3, size(T_wcam_hist, 3));
for i = 1:size(T_wcam_hist, 3)
    T_wcam =  T_wcam_hist(:, :, i);
    translation(:,i) = T_wcam(1:3, 4);
end

[Rvg,Tvg] = icp(translation,rtk_data.xyz');
rtk_registered = Rvg*rtk_data.xyz';
rtk_pos = rtk_registered;
vio_pos = translation;
[meanRMSE, transError] = calcGPSRMSE(rtk_pos, gpsTimeStamps, vio_pos, camTimeStamps);

learnedPredSpace.predVectors = [learnedPredSpace.predVectors usedPredVectors];
learnedPredSpace.weights = [learnedPredSpace.weights meanRMSE*ones(1, size(usedPredVectors,2))];


end

learnedPredSpace

%% Save
f = strsplit(rosBagFileName, '/');
f = strsplit(char(f(end)), '.');
fileName = ['learnedProbeModels/' char(f(1)) '_learnedPredSpaceIter' int2str(repeatIter) '.mat'];
save(fileName, 'learnedPredSpace');

%% Plot Trajectories 
totalDist = 0;
p_wcam_w_gt = NaN(3, size(rtk_data.xyz',2));
for j = size(rtk_data.xyz',2)
    if j > 1
        totalDist = totalDist + norm(rtk_data.xyz(j,:) - rtk_data.xyz(j-1,:));
    end
end

figure
for p_i = 1:size(p_wcam_hist,3)
    plot(p_wcam_hist(1,:,p_i),p_wcam_hist(3,:,p_i), '*b', 'LineWidth', 0.25);
    hold on;
end
%plot(rtk_data.xyz(:,1),p_wcam_w_gt(3,:), '-r', 'LineWidth', 2);
f = strsplit(rosBagFileName, '/');
f = strsplit(char(f(end)), '.');
fileName = char(f(1));

title(sprintf('Training Runs \n %s', fileName), 'Interpreter', 'none')
xlabel('x [m]')
ylabel('z [m]')

%legend('Training Runs', 'Ground Truth')
grid on;
saveas(gcf,sprintf('plots/%s_training.fig', fileName));
save(sprintf('plots/%s_paths.mat', fileName), 'p_wcam_hist', 'p_wcam_w_gt');


%%


% plot(translation(1,:),  translation(3,:),'-g', 'LineWidth', 2);
% grid on;
% hold on;


%plot(rtk_registered(1,:), rtk_registered(3,:),'-k', 'LineWidth', 2);
%title(fileName);
%xlabel('Right [m]');
%ylabel('Forward [m]');
%legend('VIO', 'RTK GPS', 'Location','NorthWest');



%saveas(gcf, ['groundtruth/' fileName '_VIOandGPS.fig']);


