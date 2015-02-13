% demonstrates stereo visual odometry on an image sequence
disp('===========================');
addpath('vo');
addpath('extraction/utils');
addpath('extraction/utils/devkit');
addpath('utils');
dataBaseDir = '/Volumes/STARSExFAT/KITTI/2011_09_30/2011_09_30_drive_0027_sync';
dataCalibDir = '/Volumes/STARSExFAT/KITTI/2011_09_30';

%% Get ground truth and import data
frameRange = 1:1100;
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

K= P_rect_cam1(:,1:3);
b_pix = P_rect_cam1(1,4);

cu = K(1,3);
cv = K(2,3);
fu = K(1,1);
fv = K(2,2);
b = -b_pix/fu; %The KITTI calibration supplies the baseline in units of pixels


param.f     = fu;
param.cu    = cu;
param.cv    = cv;
param.base  = b;
calibParams.c_u = cu;
calibParams.c_v = cv;
calibParams.f_u = fu;
calibParams.f_v = fv;
calibParams.b = b;


%%
% demonstrates sparse scene flow (quad matching = 2 consecutive stereo pairs)
% matching parameters
visualOdometryStereoMex('init',param);

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
I1 = uint8(leftImageData.rectImages(:,:,1));
I2 = uint8(rightImageData.rectImages(:,:,1));
matcherMex('push',I1,I2); 

numFrames = size(leftImageData.rectImages, 3);
k =1;
T_wcam = eye(4);
T_wcam_hist = T_wcam;

for frame=2:skipFrames:numFrames
  
  % 1-index
  
  % read current images
  I1 = uint8(leftImageData.rectImages(:,:,frame));
    I2 = uint8(rightImageData.rectImages(:,:,frame));
 
  %Show image
  axes(ha1); cla;
  imagesc(I1);
  axis off;
  
  
    T_21_opt = visualOdometryStereoMex('process',I1,I2);

        T_wcam = T_wcam*inv(T_21_opt);
        T_wcam_hist(:,:,end+1) = T_wcam;
        % update trajectory
         axes(ha2);
        plot(T_wcam(1,4),T_wcam(3,4),'g*');
         hold on;
         grid on;
         
         drawnow();
  k = k + 1
end

% release visual odometry
visualOdometryStereoMex('close');

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

%%
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
totalDist
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

%%
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

