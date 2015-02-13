% demonstrates stereo visual odometry on an image sequence
disp('===========================');
addpath('vo');
addpath('~/Dropbox/PhD/Code/MATLAB/matlab_rosbag-0.4-mac64/');
rosBagFileName = '~/Desktop/Pioneer-VI/2015-01-28-16-19-56_2loops.bag';
viRightCamTopic = '/right/image_rect';
viLeftCamTopic = '/left/image_rect';
viconTopic = '/vicon/Pioneer1/Base';

bag = ros.Bag.load(rosBagFileName);
bag.info()
bagImageRightVIData = bag.readAll({viRightCamTopic});
bagImageLeftVIData = bag.readAll({viLeftCamTopic});
viLeftcalib = bag.readAll({'/cam0/camera_info'}); %right
viRightcalib = bag.readAll({'/cam1/camera_info'}); %left
bagViconData = bag.readAll({viconTopic});

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
addpath('utils');
%settings_dataset3
%settings_KITTI;
settings_viSensor;

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

 
k =1;
T_wcam = eye(4);
for frame=first_frame+1:1:last_frame
  
  % 1-index
  
  % read current images
   I1 = uint8(reshape(bagImageLeftVIData{frame}.data, viImageSize(1), viImageSize(2))');
   I2 = uint8(reshape(bagImageRightVIData{frame}.data, viImageSize(1), viImageSize(2))');

     
   I1 = I1(1:478, :);
   I2 = I2(1:478, :);
 
  %Show image
  axes(ha1); cla;
  imagesc(I1);
  axis off;
  
  
  
    matcherMex('push',I1,I2); 
    % match images
    matcherMex('match',2);
    p_matched = matcherMex('get_matches',2);
% show matching results
disp(['Number of matched points: ' num2str(length(p_matched))]);

% showMatchedFeatures(I1, I2, p_matched(1:2,:)', p_matched(3:4,:)');
% drawnow;

        [p_f1_1, p_f2_2] = triangulateAllPointsDirect(p_matched, calibParams);
        
        pruneId = isinf(p_f1_1(1,:)) | isinf(p_f1_1(2,:)) | isinf(p_f1_1(3,:)) | isinf(p_f2_2(1,:)) | isinf(p_f2_2(2,:)) | isinf(p_f2_2(3,:));
        p_f1_1 = p_f1_1(:, ~pruneId);
        p_f2_2 = p_f2_2(:, ~pruneId);
        
        selectIdx = randperm(size(p_f1_1,2),100);

        p_f1_1 = p_f1_1(:, selectIdx);
        p_f2_2 = p_f2_2(:, selectIdx);
        
        %[p_f1_1, p_f2_2, T_21_cam_est] = findInliersRANSAC(p_f1_1, p_f2_2,optParams);
       
        R_1 = repmat(R, [1 1 size(p_f1_1, 2)]);
        R_2 = R_1;
        T_21_opt = matrixWeightedPointCloudAlignment(p_f1_1, p_f2_2, R_1, R_2, eye(4), calibParams, optParams);

        T_wcam = T_wcam*inv(T_21_opt);
        % update trajectory
         axes(ha2);
        plot3(T_wcam(1,4),T_wcam(2,4),T_wcam(3,4),'g*');
         hold on;
         grid on;
         
         drawnow();
  k = k + 1
end

% close matcher
matcherMex('close');

%%
addpath('extraction/utils')
figure
T_p1_hist = [];
for v_i = 1:length(bagViconData)
    t = bagViconData{v_i}.transform.translation;
    q = bagViconData{v_i}.transform.rotation;
    if v_i == 0
        T_g1 = [quatToRotMat(q)' t; 0 0 0 1]; 
        T_p1_hist(:,:, v_i) = eye(4);
    else
        T_p1_hist(:,:, v_i) = [quatToRotMat(q)' t; 0 0 0 1];
    end
end

t_p1_hist = [];
for v_i = 1:length(bagViconData)
    t_p1_hist(:, v_i) = T_p1_hist(1:3, 4, v_i);
end
plot(t_p1_hist(1,:), t_p1_hist(2,:), '*')


