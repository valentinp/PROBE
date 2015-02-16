kittiRun = '2011_09_26_drive_0005_sync';

load([kittiRun '_paths.mat']);
load([kittiRun '_probePath.mat']);
n6 = load([kittiRun '_noMore600Path.mat']);
n6w = load([kittiRun '_noMore600PathAndWeights.mat']);




figure
for p_i = 1:size(p_wcam_hist,3)
    h1 = plot(p_wcam_hist(1,:,p_i),p_wcam_hist(3,:,p_i), '-k', 'LineWidth', 1);
    hold on;
end
h2 = plot(translation(1,:),translation(3,:), '-g', 'LineWidth', 2);
h3 = plot(p_wcam_w_gt(1,:),p_wcam_w_gt(3,:), '-r', 'LineWidth', 2);
h4 = plot(n6.translation(1,:),n6.translation(3,:), '-b', 'LineWidth', 2);
h5 = plot(n6w.translation(1,:),n6w.translation(3,:), '-c', 'LineWidth', 2);


f = strsplit(dataBaseDir, '/');
f = strsplit(char(f(end)), '.');
fileName = char(f(1));

title(sprintf('Training Runs \n %s', fileName), 'Interpreter', 'none')
xlabel('x [m]')
ylabel('z [m]')
xlim([-30 10])

legend([h1, h2, h4, h5, h3], {'Training Runs', 'Probe Path','Exclude Right Side', 'Exclude Right Side + Probe','Ground Truth'})
grid on;
addpath('/Users/valentinp/Research/MATLAB/export_fig'); %Use Oliver Woodford's awesome export_fig package to get trimmed PDFs
export_fig(gcf, sprintf('%s_comp.pdf', kittiRun), '-transparent');

