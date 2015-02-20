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
