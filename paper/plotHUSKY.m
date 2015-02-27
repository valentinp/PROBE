%% Set up utilities
clear;
addpath('/Users/valentinp/Research/MATLAB/export_fig'); %Use Oliver Woodford's awesome export_fig package to get trimmed PDFs
fontSize = 14;

%% Plot HUSKY training
%0005,0046, 0015
fileName = '2015-02-18-12-43-18_paths.mat';

data = load(['../plots/' fileName]);

figure
for p_i = 1:size(data.p_wcam_hist,3)
    useIdx = data.p_wcam_hist(1,:,p_i) > 0;
    p_wcam_hist = data.p_wcam_hist(:,useIdx,p_i);
    h1 = plot(p_wcam_hist(1,:),p_wcam_hist(3,:), '--b', 'LineWidth', 2);
    hold on;
end

% Calculate GPS Time
addpath('../groundtruth/');
gpsFile = '/Users/valentinp/Desktop/Pioneer-VI/GPS/rover3_rtk.mat';
rtk_data = load(gpsFile);
gpsStartTime = rtk_data.initialTime;
gpsPointsNum = size(rtk_data.xyz,1);
timeComponents = strsplit(gpsStartTime, ':');
timeVec = [2015,2,18,str2num(timeComponents{1}),str2num(timeComponents{2}), str2num(timeComponents{3})];
gpsStartMatlabDateNum = datenum(timeVec);
datestr(gpsStartMatlabDateNum)
gpsStartUnixTime = double((gpsStartMatlabDateNum - 719529)*86400 + 5*3600);
gpsTimeStamps = double(1:gpsPointsNum) + gpsStartUnixTime;
[Rvg,Tvg] = icp(p_wcam_hist,rtk_data.xyz');
rtk_registered = Rvg*rtk_data.xyz';

h2 = plot(rtk_registered(1,:),rtk_registered(3,:), '-k', 'LineWidth', 2);
%stitle(sprintf('Training Runs \n %s', fileName), 'Interpreter', 'none')
% xlabel('x [m]')
% ylabel('z [m]')
title('Training');
fontSize = 30;
set(gca, 'FontSize', fontSize);
ylim([0 80])
xlim([-10 100])
f = strsplit(fileName, '.');
fileTitle = f{1};
%legend([h1, h2], {'Training Runs', 'Ground Truth Path'}, 'Location', 'SouthEast');
grid on;

export_fig(gcf, sprintf('%s_small.pdf', fileTitle), '-transparent');

%% Run Comparison

%%  City
fontSize = 18;
huskyRun = '2015-02-18-12-33-30';
dataNom = load(['../trials/' huskyRun '_nominal.mat']);
dataAgg = load(['../trials/' huskyRun '_aggressive.mat']);
dataProbe = load(['../trials/' huskyRun '_probe.mat']);
figure

hgt = plot(dataNom.rtk_registered(1,:),dataNom.rtk_registered(3,:), '-k', 'LineWidth', 2);
hold on;
hnom = plot(dataNom.p_camw_w(1,:),dataNom.p_camw_w(3,:), '-.r', 'LineWidth', 2);
hagg = plot(dataAgg.p_camw_w(1,:),dataAgg.p_camw_w(3,:), '--g', 'LineWidth', 2);
hprobe = plot(dataProbe.p_camw_w(1,:),dataProbe.p_camw_w(3,:), '-b', 'LineWidth', 2);
xlabel('x [m]')
ylabel('z [m]')
set(gca, 'FontSize', fontSize);
f = strsplit(fileName, '.');
fileTitle = f{1};
legend([hnom, hagg,hprobe, hgt], {'Nominal', 'Aggressive', 'PROBE', 'Ground Truth'}, 'Location', 'NorthWest');
grid on;
export_fig(gcf, sprintf('%s_comparison.pdf', huskyRun), '-transparent');

norm(dataNom.p_camw_w(:,end) - dataNom.rtk_registered(:,end))
norm(dataAgg.p_camw_w(:,end) - dataAgg.rtk_registered(:,end))
norm(dataProbe.p_camw_w(:,end) - dataProbe.rtk_registered(:,end))

