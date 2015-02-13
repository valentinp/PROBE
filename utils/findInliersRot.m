function [p_f1_1, p_f2_2] = findInliersRot(p_f1_1, p_f2_2, C_21, optParams)
%FINDINLIERS Use a threshold to remove outliers

    inlierSet = [];
    costThresh = optParams.RANSACCostThresh;
    for f_i = 1:size(p_f1_1,2)
        cost = acosd((C_21*p_f1_1(:,f_i))'*p_f2_2(:,f_i)/(norm(C_21*p_f1_1(:,f_i))*norm(p_f2_2(:,f_i))));
        if cost < costThresh
            inlierSet(end+1) = f_i;
        end
    end
    p_f1_1 = p_f1_1(:, inlierSet);
    p_f2_2 = p_f2_2(:, inlierSet);
end

