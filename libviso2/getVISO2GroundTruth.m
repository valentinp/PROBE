% demonstrates stereo visual odometry on an image sequence
disp('===========================');
addpath('~/Dropbox/PhD/Code/MATLAB/matlab_rosbag-0.4-mac64/');
rosBagFileName = '/Volumes/STARSExFAT/IROSData/2015-02-17-13-23-44.bag';
viRightCamTopic = '/right/image_rect';
viLeftCamTopic = '/left/image_rect';

bag = ros.Bag.load(rosBagFileName);
bag.info()
bagImageRightVIData = bag.readAll({viRightCamTopic});
bagImageLeftVIData = bag.readAll({viLeftCamTopic});
viLeftcalib = bag.readAll({'/cam0/camera_info'}); %right
viRightcalib = bag.readAll({'/cam1/camera_info'}); %left
viImageSize = [752 480];

data = load('datasets/2015-01-28-16-11-17_1loop.mat');
param.f     = data.fu;
param.cu    = data.cu;
param.cv    = data.cv;
param.base  = data.b;
first_frame = 1;
last_frame  = length(bagImageRightVIData);

% init visual odometry
visualOdometryStereoMex('init',param);

% init transformation matrix array
Tr_total{1} = eye(4);

% create figure
figure('Color',[1 1 1]);
ha1 = axes('Position',[0.05,0.7,0.9,0.25]);
%axis off;
ha2 = axes('Position',[0.05,0.05,0.9,0.6]);
set(gca,'XTick',-500:10:500);
set(gca,'YTick',-500:10:500);
axis equal, grid on, hold on;

% for all frames do
k =1;
for frame=first_frame:last_frame
  
  % 1-index
  
  % read current images
   I1 = uint8(reshape(bagImageLeftVIData{frame}.data, viImageSize(1), viImageSize(2))');
   I2 = uint8(reshape(bagImageRightVIData{frame}.data, viImageSize(1), viImageSize(2))');

     
   I1 = I1(1:478, :);
   I2 = I2(1:478, :);
   
  % compute and accumulate egomotion
  Tr = visualOdometryStereoMex('process',I1,I2);
  if k>1
    Tr_total{k} = Tr_total{k-1}*inv(Tr);
  end

  % update image
  axes(ha1); cla;
  imagesc(I1); colormap(gray);
  axis off;
  
  % update trajectory
  axes(ha2);
  if k>1
    plot3([Tr_total{k-1}(1,4) Tr_total{k}(1,4)], [Tr_total{k-1}(2,4) Tr_total{k}(2,4)], ...
         [Tr_total{k-1}(3,4) Tr_total{k}(3,4)],'-xb','LineWidth',1);
  end
  pause(0.01); drawnow;

  % output statistics
  num_matches = visualOdometryStereoMex('num_matches');
  num_inliers = visualOdometryStereoMex('num_inliers');
  disp(['Frame: ' num2str(frame) ...
        ', Matches: ' num2str(num_matches) ...
        ', Inliers: ' num2str(100*num_inliers/num_matches,'%.1f') ,' %']);
  k = k + 1;
end

% release visual odometry
visualOdometryStereoMex('close');
%%
norm(Tr_total{end}(1:3, 4))
Tr_total{end}(1:3, 4)

p_camw_w_gt = NaN(3, length(Tr_total));
for i = 1:length(Tr_total)
    p_camw_w_gt(:, i) = Tr_total{i}(1:3,4);
end

f = strsplit(rosBagFileName, '/');
f = strsplit(char(f(end)), '.');
fileName = [char(f(1)) '_GroundTruth.mat'];
save(fileName, 'p_camw_w_gt');


