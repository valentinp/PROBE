function [RMSE, transError] = calcGPSRMSE(rtk_pos, rtk_time, vio_pos, vio_time)
%Assumes 3xN positions and 1xN timestamps
transError = NaN(3, size(rtk_pos, 2));
for p_i = 1:size(rtk_pos, 2)
    [ idx ] = findClosestTimestamp(rtk_time(p_i), vio_time);
    transError(:,p_i) = rtk_pos(:,p_i) - vio_pos(:,idx);
end
RMSE = mean(sqrt(sum(transError.^2,1)/3));
end