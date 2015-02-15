function [p_f1_1, p_f2_2,T_21_cam_best] = findInliersRot(p_f1_1, p_f2_2, C_21, optParams, calibParams)
%FINDINLIERS 

if size(p_f1_1, 2) < 4
    T_21_cam_best = scalarWeightedPointCloudAlignment(p_f1_1, p_f2_2, C_21);
    return;
end



ransacMaxIterations = 50;
numPoints = size(p_f1_1, 2);
maxInlierCount = 3;
bestInlierSet = 1:numPoints;

for rn_i = 1:ransacMaxIterations
    %Select three random points
    threeIdx = randperm(numPoints,3);
    restIdx = setdiff(1:numPoints, threeIdx);
    
   T_21_test = scalarWeightedPointCloudAlignment(p_f1_1(:,threeIdx), p_f2_2(:,threeIdx),C_21);

    
    %Compute all points that fall outside the threshold (via cost function)
    inlierCount = 0;
    inlierSet = threeIdx;
%     costHist = NaN(1, length(restIdx));
    for f_i = restIdx
        errorVec = homo2cart(T_21_test*cart2homo(p_f1_1(:,f_i))) - p_f2_2(:,f_i);
        cost = 0.5*(errorVec)'*(errorVec);
%         costHist(f_i) = cost;
        if cost < optParams.RANSACCostThresh
           inlierCount = inlierCount + 1;
           inlierSet(end+1) = f_i;
        end
    end
%     hist(costHist);
%     pause();
    
    if inlierCount > maxInlierCount
        maxInlierCount = inlierCount;
        bestInlierSet = inlierSet;
    end
    
end

p_f1_1 = p_f1_1(:,bestInlierSet);
p_f2_2 = p_f2_2(:,bestInlierSet);

T_21_cam_best = scalarWeightedPointCloudAlignment(p_f1_1, p_f2_2, C_21);

fprintf('%d inliers. %d outliers \n', length(bestInlierSet), numPoints - length(bestInlierSet));
end

