%% Clean up and import
clc;
close all;
addpath('utils');
addpath('utils/devkit');


dataBaseDir = '~/Desktop/KITTI/2011_09_26/2011_09_26_drive_0005_sync';
dataCalibDir = '~/Desktop/KITTI/2011_09_26';

%% Get ground truth and import data
frameRange = 1:154;
%Image data
leftImageData = loadImageData([dataBaseDir '/image_00'], frameRange);
rightImageData = loadImageData([dataBaseDir '/image_01'], frameRange);
%IMU data
[imuData, imuFrames] = loadImuData(dataBaseDir, leftImageData.timestamps);
%Ground Truth
T_wIMU_GT = getGroundTruth(dataBaseDir, imuFrames);

skipFrames = 1;
minObsNum = 3;

%% Load calibration
[T_camvelo_struct, P_rect_cam1] = loadCalibration(dataCalibDir);
T_camvelo = T_camvelo_struct{1}; 
T_veloimu = loadCalibrationRigid(fullfile(dataCalibDir,'calib_imu_to_velo.txt'));
T_camimu = T_camvelo*T_veloimu;

%Add camera ground truth

T_wCam_GT = T_wIMU_GT;

for i = 1:size(T_wIMU_GT, 3)
    T_wCam_GT(:,:,i) = T_wIMU_GT(:,:,i)*inv(T_camimu);
end

%% Process images

meanErrorHist = [];
errorVecHist = [];
bmHist = [];
imuDataHist = [];

seenFeatureVectors = [];
seenFeatureStructs = {};
trackedFeatureIdx = [];

numFrames = size(leftImageData.rectImages, 3);
detectNewPoints = false;
for i=1:skipFrames:numFrames
    fprintf('Processing image %d \n', i);
    %Extract stereo images
    
    viLeftImage = uint8(leftImageData.rectImages(:,:,i));
    viRightImage = uint8(rightImageData.rectImages(:,:,i));
    
    if i == 1 || detectNewPoints
    detectNewPoints = false;     
    %Detect strongest corners
    leftPoints = detectSURFFeatures(viLeftImage);
    rightPoints = detectSURFFeatures(viRightImage);
    
    leftPoints = leftPoints.selectStrongest(500);
    rightPoints = rightPoints.selectStrongest(500);
    
    %Extract features and stereo match
   [featuresLeft, validLeftPoints] = extractFeatures(viLeftImage, leftPoints);
   [featuresRight, validRightPoints] = extractFeatures(viRightImage, rightPoints);

    indexPairs = matchFeatures(featuresLeft, featuresRight);
    matchedPointsLeft = validLeftPoints(indexPairs(:, 1), :);
    featuresLeft = featuresLeft(indexPairs(:, 1), :);
    matchedPointsRight = validRightPoints(indexPairs(:, 2), :);
    
    inliers = abs((matchedPointsLeft.Location(:, 2) - matchedPointsRight.Location(:, 2))) <= 0.25 & abs((matchedPointsLeft.Location(:, 1) - matchedPointsRight.Location(:, 1))) > 3;
    
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
         & validRightPoints(:,1) > 0 & validRightPoints(:,2) > 0 ...
         & abs(validLeftPoints(:, 2) - validRightPoints(:, 2)) <= 1);
     
     if length(trackedIdx) < 10
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

pruneIds = [];
totObs = 0;
for p_i = 1:length(seenFeatureStructs)
    totObs = totObs + length(seenFeatureStructs{p_i}.imageIndex);
    trackLength = norm(seenFeatureStructs{p_i}.leftPixels(:, 1) - seenFeatureStructs{p_i}.leftPixels(:, end));
    
    lp = abs(diff(sum(seenFeatureStructs{p_i}.leftPixels, 1)));
    rp = abs(diff(sum(seenFeatureStructs{p_i}.leftPixels, 1)));
    
    maxPixDiff = max([lp rp]);
    minPixDiff = min([lp rp]);

    if length(seenFeatureStructs{p_i}.imageIndex) < minObsNum || trackLength < 5 || maxPixDiff > 50 || minPixDiff < 1
        pruneIds(end+1) = p_i;
    end
end

fprintf('%d features to prune. \n',length(pruneIds));
fprintf('%d remaining features. \n', length(seenFeatureStructs) - length(pruneIds));

seenFeatureStructsPruned = removeCells(seenFeatureStructs, pruneIds);

%% Plot feature tracks

figure
subplot(2,1,1);
plot(0,0);
hold on;
for f_i = 1:length(seenFeatureStructsPruned)
    plot(seenFeatureStructsPruned{f_i}.leftPixels(1,:), seenFeatureStructsPruned{f_i}.leftPixels(2,:));
end
subplot(2,1,2);
plot(0,0);
hold on;
for f_i = 1:length(seenFeatureStructsPruned)
    plot(seenFeatureStructsPruned{f_i}.rightPixels(1,:), seenFeatureStructsPruned{f_i}.rightPixels(2,:));
end

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

%% Export variables
w_vk_vk_i = imuData.measOmega;
v_vk_vk_i = imuData.measVel;

K= P_rect_cam1(:,1:3);
b_pix = P_rect_cam1(1,4);

cu = K(1,3);
cv = K(2,3);
fu = K(1,1);
fv = K(2,2);
b = -b_pix/fu; %The KITTI calibration supplies the baseline in units of pixels


R_ci = T_camimu(1:3,1:3);
T_imucam = inv(T_camimu);
p_ci_i = T_imucam(1:3,4)


C_c_v = R_ci;
rho_v_c_v = p_ci_i;

f = strsplit(dataBaseDir, '/');
f = strsplit(char(f(end)), '.');
fileName = [char(f(1)) '_KLT.mat'];

size(w_vk_vk_i)
size(v_vk_vk_i)
size(y_k_j)

p_vi_i = NaN(3, size(T_wIMU_GT,3));
for j = 1:size(T_wIMU_GT,3)
p_vi_i(:,j) = T_wIMU_GT(1:3, 4, j);
end

t = leftImageData.timestamps;
save(fileName, 'p_vi_i','w_vk_vk_i','v_vk_vk_i', 'cu','cv','fu','fv','b', 'y_k_j', 'C_c_v', 'rho_v_c_v', 't');

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
