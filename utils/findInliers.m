function [p_f1_1, p_f2_2] = findInliers(p_f1_1, p_f2_2, T_21)
%FINDINLIERS Use a threshold to remove outliers

    inlierSet = [];
    costThresh = 0.2;
    for f_i = 1:size(p_f1_1,2)
        errorVec = homo2cart(T_21*cart2homo(p_f1_1(:,f_i))) - p_f2_2(:,f_i);
        cost = 0.5*(errorVec)'*(errorVec);
        if cost < costThresh
            inlierSet(end+1) = f_i;
        end
    end
    p_f1_1 = p_f1_1(:, inlierSet);
    p_f2_2 = p_f2_2(:, inlierSet);
end

