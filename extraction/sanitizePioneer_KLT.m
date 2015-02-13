%% Clean up and import
clc;
close all;
%addpath('YAMLMatlab');
addpath('utils');
addpath('integration');

if ismac
    addpath('~/Dropbox/PhD/Code/MATLAB/matlab_rosbag-0.4-mac64/');
else
    addpath('~/Dropbox/PhD/Code/MATLAB/matlab_rosbag-0.4-linux64/');
end
%% Parameters
rosBagFileName = '~/Desktop/Pioneer-VI/2015-01-28-16-11-17_1loop.bag';
imuTopic = '/imu0';
viRightCamTopic = '/right/image_rect';
viLeftCamTopic = '/left/image_rect';
viconTopic = '/vicon/Pioneer1/Base';
viImageSize = [752 480];

skipFrames = 1;
minObsNum = 2;
imuRate = 200;

%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
bag = ros.Bag.load(rosBagFileName);
bag.info()
%% Use YAML File 
%yaml_file = '/Users/valentinp/Desktop/Tricifix-VI/camchain-homevalentinDesktopTricific-VI2015-01-22-14-45-52_extcalib.yaml';
%YamlStruct = ReadYaml(yaml_file);

%% Read all messages 
bagImageRightVIData = bag.readAll({viRightCamTopic});
bagImageLeftVIData = bag.readAll({viLeftCamTopic});
bagImuData = bag.readAll({imuTopic});
bagViconData = bag.readAll({viconTopic});

%% Load Calibration
viLeftcalib = bag.readAll({'/cam0/camera_info'}); %right
viRightcalib = bag.readAll({'/cam1/camera_info'}); %left

%% Process images

meanErrorHist = [];
errorVecHist = [];
bmHist = [];
imuDataHist = [];

seenFeatureVectors = [];
seenFeatureStructs = {};
trackedFeatureIdx = [];

numFrames = length(bagImageLeftVIData);
detectNewPoints = false;
for i=1:skipFrames:numFrames
    fprintf('Processing image %d \n', i);
    %Extract stereo images
    
     viLeftImage = reshape(bagImageLeftVIData{i}.data, viImageSize(1), viImageSize(2))';
    viRightImage = reshape(bagImageRightVIData{i}.data, viImageSize(1), viImageSize(2))';
    
       viLeftImage = viLeftImage(1:478, :);
   viRightImage = viRightImage(1:478, :);
   
    if i == 1 || detectNewPoints
    detectNewPoints = false;     
    %Detect strongest corners
    leftPoints = detectSURFFeatures(viLeftImage);
    rightPoints = detectSURFFeatures(viRightImage);
    
    leftPoints = leftPoints.selectStrongest(300);
    rightPoints = rightPoints.selectStrongest(300);
    
    %Extract features and stereo match
   [featuresLeft, validLeftPoints] = extractFeatures(viLeftImage, leftPoints);
   [featuresRight, validRightPoints] = extractFeatures(viRightImage, rightPoints);

    indexPairs = matchFeatures(featuresLeft, featuresRight);
    matchedPointsLeft = validLeftPoints(indexPairs(:, 1), :);
    featuresLeft = featuresLeft(indexPairs(:, 1), :);
    matchedPointsRight = validRightPoints(indexPairs(:, 2), :);
    
    inliers = abs((matchedPointsLeft.Location(:, 2) - matchedPointsRight.Location(:, 2))) <= 1;
    
    matchedPointsLeft = matchedPointsLeft(inliers, :);
    matchedPointsRight = matchedPointsRight(inliers, :);
   
     pointTrackerL = vision.PointTracker('MaxBidirectionalError', 5);
     initialize(pointTrackerL, matchedPointsLeft.Location, viLeftImage);
     pointTrackerR = vision.PointTracker('MaxBidirectionalError', 5);
     initialize(pointTrackerR, matchedPointsRight.Location, viRightImage);
   
     
      %For all new features,add a new struct
        fCount = length(seenFeatureStructs) + 1;
        trackedFeatureIdx = [];
        for obs_i = 1:size(matchedPointsLeft.Location,1)
            seenFeatureStructs{fCount}.leftPixels = matchedPointsLeft.Location(obs_i, :)';
            seenFeatureStructs{fCount}.rightPixels = matchedPointsRight.Location(obs_i, :)';
            seenFeatureStructs{fCount}.imageIndex = i;
            trackedFeatureIdx(end+1) = fCount;
            fCount = fCount + 1;
        end
     
    else
       
     [validLeftPoints, isFoundL] = step(pointTrackerL, viLeftImage);
     [validRightPoints, isFoundR] = step(pointTrackerR, viRightImage);
     
     trackedIdx = find(isFoundL==isFoundR & validLeftPoints(:,1) > 0 & validLeftPoints(:,2) > 0 ...
         & validRightPoints(:,1) > 0 & validRightPoints(:,2) > 0);
     
     if length(trackedIdx) < 50
         detectNewPoints = true;
     end
     if isempty(trackedIdx)
        continue;
     end
     
     fprintf('Tracking %d features.', length(trackedIdx));
     
        setPoints(pointTrackerL, validLeftPoints(trackedIdx,:));
        setPoints(pointTrackerR, validRightPoints(trackedIdx,:)); 
    
     observedIdx = trackedFeatureIdx(trackedIdx);
     trackedFeatureIdx= trackedFeatureIdx(trackedIdx);
     
    %For all previously seen features, update the feature struct to account
    %for new observation
        for f_i = 1:length(observedIdx)
            struct_i = observedIdx(f_i);
            obs_i = trackedIdx(f_i);
            seenFeatureStructs{struct_i}.leftPixels(:, end+1) = validLeftPoints(obs_i, :)';
            seenFeatureStructs{struct_i}.rightPixels(:, end+1) = validRightPoints(obs_i, :)';
            seenFeatureStructs{struct_i}.imageIndex(end+1) = i;
        end
  
    
       showMatchedFeatures(viLeftImage, viRightImage, validLeftPoints(trackedIdx,:), validRightPoints(trackedIdx,:));
   end
   
  
    %=====Temporal tracking=======

%
   
end

%% Prune points with low observation count
% 
pruneIds = [];
totObs = 0;
for p_i = 1:length(seenFeatureStructs)
    totObs = totObs + length(seenFeatureStructs{p_i}.imageIndex);
    trackLength = norm(seenFeatureStructs{p_i}.leftPixels(:, 1) - seenFeatureStructs{p_i}.leftPixels(:, end));
    
    lp = abs(diff(sum(seenFeatureStructs{p_i}.leftPixels, 1)));
    rp = abs(diff(sum(seenFeatureStructs{p_i}.leftPixels, 1)));
    
    maxPixDiff = max([lp rp]);
    minPixDiff = min([lp rp]);

    if length(seenFeatureStructs{p_i}.imageIndex) < minObsNum
        pruneIds(end+1) = p_i;
    end
end

fprintf('%d features to prune. \n',length(pruneIds));
fprintf('%d remaining features. \n', length(seenFeatureStructs) - length(pruneIds));

seenFeatureStructsPruned = removeCells(seenFeatureStructs, pruneIds);

% %% Plot feature tracks
% 
% figure
% subplot(2,1,1);
% plot(0,0);
% hold on;
% for f_i = 1:length(seenFeatureStructsPruned)
%     plot(seenFeatureStructsPruned{f_i}.leftPixels(1,:), seenFeatureStructsPruned{f_i}.leftPixels(2,:));
% end
% subplot(2,1,2);
% plot(0,0);
% hold on;
% for f_i = 1:length(seenFeatureStructsPruned)
%     plot(seenFeatureStructsPruned{f_i}.rightPixels(1,:), seenFeatureStructsPruned{f_i}.rightPixels(2,:));
% end

%%

% y_k_j, 4 x K x 20
y_k_j = -1*ones(4, length(1:skipFrames:numFrames), length(seenFeatureStructsPruned));

data_i = 1;
for image_i = 1:skipFrames:numFrames
    fprintf('Pruning features from image %d \n', image_i);
    for p_i = 1:length(seenFeatureStructsPruned)
        [isObs, idx] = ismember(image_i, seenFeatureStructsPruned{p_i}.imageIndex);
        if isObs
            y_k_j(:, data_i, p_i) = [seenFeatureStructsPruned{p_i}.leftPixels(:,idx); seenFeatureStructsPruned{p_i}.rightPixels(:,idx)];
        end
    end
    data_i = data_i + 1;
end


%%
%Integrate IMU measurements
addpath('integration');
gVec = zeros(3,1);

%Calculate gravity
for imu_i=1:imuRate*5
    gVec = gVec + 1/(imuRate*5)*[bagImuData{imu_i}.linear_acceleration];
end
%Subtract gravity
imuData = NaN(6, length(bagImuData));
imuDataTimeStamps = NaN(1, length(bagImuData));
for imu_i=1:length(bagImuData)
    imuDataTimeStamps(imu_i) = bagImuData{imu_i}.header.stamp.time;
    imuData(:, imu_i) = [bagImuData{imu_i}.linear_acceleration - gVec; bagImuData{imu_i}.angular_velocity];
end

%Zero out out of plane measurements
% imuData(2,:) = zeros(length(imuData(2,:)), 1);
% imuData(4,:) = zeros(length(imuData(4,:)), 1);
% imuData(6,:) = zeros(length(imuData(6,:)), 1);

%Begin integration
lastViTime = bagImageLeftVIData{1}.header.stamp.time;
xInit.p = zeros(3,1);
xInit.v = zeros(3,1);
xInit.b_g = zeros(3,1);
xInit.b_a = zeros(3,1);
xInit.q = [1; zeros(3,1)];
xState = xInit;


imuSanitizedData = NaN(6, length(1:skipFrames:numFrames));

viTime = NaN(1, length(1:skipFrames:numFrames));

data_i = 1;
for image_i = 1:skipFrames:numFrames
    fprintf('Processing IMU data from image %d \n', image_i);
    currentViTime = bagImageLeftVIData{image_i}.header.stamp.time;
    viTime(data_i) = currentViTime;
    mask = imuDataTimeStamps >= lastViTime & imuDataTimeStamps < currentViTime;
    if sum(mask) == 0
        mask(1) = 1;
    end
    imuDataWindow = imuData(:, mask);
    imuDataWindowTimeStamps = imuDataTimeStamps(mask);
    dt_vi = currentViTime - lastViTime;
    lastViTime = currentViTime;
    
    %Take average of angular velocities
    angVel = sum(imuDataWindow(4:6, :),2)/size(imuDataWindow,2);
    
    %Integrate linear accelerations
    for imusub_i = 1:size(imuDataWindow,2)
        if imusub_i == 1
            dt = 1/200;
        else
            dt = imuDataWindowTimeStamps(imusub_i) - imuDataWindowTimeStamps(imusub_i-1);
        end
        [xState] = integrateIMU(xState, imuDataWindow(1:3, imusub_i), imuDataWindow(4:6, imusub_i), dt, zeros(3,1));
    end
    imuSanitizedData(:, data_i) = [xState.v; angVel];
    data_i = data_i + 1;
end

%% Export variables
w_vk_vk_i = imuSanitizedData(4:6, :);
v_vk_vk_i = imuSanitizedData(1:3, :);

viIntrinsics = [viLeftcalib{1}.P(1) viLeftcalib{1}.P(6) viLeftcalib{1}.P(3) viLeftcalib{1}.P(7)]; %f_u f_v c_u c_v
cu = viIntrinsics(3);
cv = viIntrinsics(4);
fu = viIntrinsics(1);
fv = viIntrinsics(2);
b = 0.11; %very close to 11cm


p_ci_i = [0.0602131703928; -0.00145860604554; -0.0465617957282];
R_ic = quatToRotMat([-5.44616859609e-05 0.00246256274859 -0.000268097157994 0.999996930467]');

C_c_v = R_ic';
rho_v_c_v = p_ci_i;

f = strsplit(rosBagFileName, '/');
f = strsplit(char(f(end)), '.');
fileName = [char(f(1)) '.mat'];

size(w_vk_vk_i)
size(v_vk_vk_i)
size(y_k_j)

t = viTime - viTime(1);
save(fileName, 'w_vk_vk_i','v_vk_vk_i', 'cu','cv','fu','fv','b', 'y_k_j', 'C_c_v', 'rho_v_c_v', 't');


%% Plot Stuff!

% figure 
% subplot(2,2,1)
% hist(imuSanitizedData(4, :), 50)
% subplot(2,2,2)
% hist(imuSanitizedData(5, :), 50)
% subplot(2,2,3)
% hist(imuSanitizedData(6, :),50)

% figure
% plot(imuData(4, :))
% hold on
% plot(imuData(5, :))
% plot(imuData(6, :))

% figure 
% subplot(2,2,1)
% hist(imuData(1, :), 50)
% subplot(2,2,2)
% hist(imuData(2, :), 50)
% subplot(2,2,3)
% hist(imuData(3, :),50)
% 
% 
% mean(imuData(1, :))
% mean(imuData(2, :))
% mean(imuData(3, :))
