%% Set up utilities
clear;
addpath('/Users/valentinp/Research/MATLAB/export_fig'); %Use Oliver Woodford's awesome export_fig package to get trimmed PDFs
fontSize = 14;

%% Plot PIONEER training
fileName = '2015-02-25-16-56-01_paths.mat';

data = load(['../plots/' fileName]);

figure
for p_i = 1:size(data.p_wcam_hist,3)
    p_wcam_hist = data.p_wcam_hist(:,:,p_i);
    h1 = plot(p_wcam_hist(1,:),p_wcam_hist(3,:), '--b', 'LineWidth', 2);
    hold on;
end

%xlabel('x [m]')
%ylabel('z [m]')
title('Training');
set(gca, 'FontSize', fontSize);
xlim([-5 5])
f = strsplit(fileName, '.');
fileTitle = f{1};
%legend([h1], {'Training Runs'}, 'Location', 'SouthWest');
grid on;
fontSize = 30;
export_fig(gcf, sprintf('%s_small.pdf', fileTitle), '-transparent');

%% Run Comparison
fontSize = 18;
huskyRun = '2015-02-25-16-58-23';
dataNom = load(['../trials/' huskyRun '_nominal.mat']);
dataAgg = load(['../trials/' huskyRun '_aggressive.mat']);
dataProbe = load(['../trials/' huskyRun '_probe.mat']);

figure
hold on;
hnom = plot3(dataNom.translation(1,:),dataNom.translation(2,:),dataNom.translation(3,:), '-.r', 'LineWidth', 2.5);
hagg = plot3(dataAgg.translation(1,:),dataAgg.translation(2,:),dataNom.translation(3,:), '--g', 'LineWidth', 2.5);
hprobe = plot3(dataProbe.translation(1,:),dataProbe.translation(2,:),dataNom.translation(3,:), '-b', 'LineWidth', 2.5);
view(-37,18)
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

dataNom.translation(:,1)
norm(dataNom.translation(:,1) - dataNom.translation(:,end))
norm(dataAgg.translation(:,1) - dataAgg.translation(:,end))
norm(dataProbe.translation(:,1) - dataProbe.translation(:,end))

totalDist = 0;
for i = 2:size(dataProbe.translation,2)
    totalDist = totalDist + norm(dataProbe.translation(:,i) - dataProbe.translation(:,i-1));
end
totalDist
set(gca, 'FontSize', fontSize);
f = strsplit(fileName, '.');
fileTitle = f{1};
legend([hnom, hagg,hprobe], {'Nominal', 'Aggressive', 'PROBE'}, 'Location', 'NorthWest');
grid on;
export_fig(gcf, sprintf('%s_comparison.pdf', huskyRun), '-transparent');


