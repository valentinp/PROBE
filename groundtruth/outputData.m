fileName = '2015-02-18-12-43-18_VIO';
load([fileName '.mat']);
figure
translation = NaN(3, size(T_wcam_hist, 3));
for i = 1:size(T_wcam_hist, 3)
    T_wcam =  T_wcam_hist(:, :, i);
    translation(:,i) = T_wcam(1:3, 4);
end
plot(translation(1,:), translation(3,:),'-g', 'LineWidth', 2);
grid on;
hold on;

rtk_data = load('/Users/valentinp/Desktop/Pioneer-VI/GPS/rover1_rtk.mat');

[R,T] = icp(translation,rtk_data.xyz');

rtk_registered = R*rtk_data.xyz';

%plot3(rtk_data.xyz(:,1),rtk_data.xyz(:,2), rtk_data.xyz(:,3),'-k', 'LineWidth', 2);
plot(rtk_registered(1,:), rtk_registered(3,:),'-k', 'LineWidth', 2);
xlabel('Right [m]');
ylabel('Forward [m]');
legend('VIO', 'RTK GPS', 'Location','NorthWest');


trans_cam = translation;
trans_rtk = rtk_registered;
save([fileName '_VIOandRTK.mat'], 'trans_cam', 'trans_rtk');

%%

run1 = load('/Users/valentinp/Desktop/Pioneer-VI/GPS/rover1_rtk.mat');
run2 = load('/Users/valentinp/Desktop/Pioneer-VI/GPS/rover2_rtk.mat');
run3 = load('/Users/valentinp/Desktop/Pioneer-VI/GPS/rover3_rtk.mat');

totalDist = 0;
for i = 2:size(run1.xyz,1)
    totalDist = totalDist + norm(run1.xyz(i,:) - run1.xyz(i-1,:));
end
totalDist
totalDist = 0;
for i = 2:size(run2.xyz,1)
    totalDist = totalDist + norm(run2.xyz(i,:) - run2.xyz(i-1,:));
end
totalDist
totalDist = 0;
for i = 2:size(run3.xyz,1)
    totalDist = totalDist + norm(run3.xyz(i,:) - run3.xyz(i-1,:));
end
totalDist

%%
gpsStartTime = run2.initialTime;
gpsPointsNum = size(run2.xyz,1);
timeComponents = strsplit(gpsStartTime, ':');
timeVec = [2015,2,18,str2num(timeComponents{1}),str2num(timeComponents{2}), str2num(timeComponents{3})];
gpsStartMatlabDateNum = datenum(timeVec);
datestr(gpsStartMatlabDateNum)
gpsStartUnixTime = double((gpsStartMatlabDateNum - 719529)*86400 + 5*3600);
gpsTimeStamps = double(1:gpsPointsNum) + gpsStartUnixTime;
