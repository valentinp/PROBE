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
% addpath('learning');
% bmHist = [];
% allPredVectors = [];
% for frame=first_frame+1:1:last_frame-2
%     frame
%    % read current images
%    I1 = (reshape(bagImageLeftVIData{frame}.data, viImageSize(1), viImageSize(2))');
%    %I2 = (reshape(bagImageRightVIData{frame}.data, viImageSize(1), viImageSize(2))');
%    I1 = I1(1:478, :);
%    %I2 = I2(1:478, :);
%     
%     keyPoints = detectSURFFeatures(mat2gray(I1));
%     keyPointPixels = keyPoints.Location(:,:)';
%     currentViTime = bagImageLeftVIData{frame}.header.stamp.time;
% 
%     %Computes Prediction Space points based on the image and keypoint position
%     imu_i = findClosestTimestamp(currentViTime, imuDataTimeStamps);
%     imuDataRecord = imuData(:, imu_i);
%     
%     predVectors = computePredVectors(keyPointPixels, I1, imuDataRecord);
%     allPredVectors = [allPredVectors predVectors];
% 
% end
% 
% 
% numClusters = 3;
% Km = 1.5;
% [idx, C,~,D] = kmeans(allPredVectors', numClusters);
% 
% 
% [COEFF,SCORE, latent] = princomp(allPredVectors');
% latent
% COEFF
% 
% 
% % Determine the mean distance of points within a cluster to the cluster's centroid
% % This sets boundaries
% 
% meanCentroidDists = zeros(numClusters, 1);
% predVectorsWithinBoundaries = 0;
% for ic = 1:numClusters
%     meanCentroidDists(ic) = Km*mean(D(idx == ic, ic));
%     predVectorsWithinBoundaries = predVectorsWithinBoundaries + sum(D(idx == ic, ic) < meanCentroidDists(ic));
% end
% 
% inFraction = predVectorsWithinBoundaries/size(allPredVectors,2);
% 
% clusteringModel.clusterNum = numClusters;
% clusteringModel.centroids = C';
% clusteringModel.threshDists = meanCentroidDists;
% 
% save('cluster.mat', 'clusteringModel');
% 
% 
% 
% eva = evalclusters(allPredVectors','kmeans','CalinskiHarabasz','KList',[1:10])
