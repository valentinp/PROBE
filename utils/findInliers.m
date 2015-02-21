function [p_f1_1, p_f2_2, inlierSet] = findInliers(p_f1_1, p_f2_2, C_21, optParams)
%FINDINLIERS Use a threshold to remove outliers

    inlierSet = [];
    for f_i = 1:size(p_f1_1,2)
        vec1 = C_21*p_f1_1(:,f_i);
        vec2 = p_f2_2(:,f_i);
        cost = 1 - dot(vec1,vec2)/(norm(vec1)*norm(vec2));
        if cost < optParams.OutlierThresh
            inlierSet(end+1) = f_i;
        end
    end
    if isempty(inlierSet)
        inlierSet = randperm(size(p_f1_1,2),3);
        p_f1_1 = p_f1_1(:, inlierSet);
        p_f2_2 = p_f2_2(:, inlierSet);
    else
        p_f1_1 = p_f1_1(:, inlierSet);
        p_f2_2 = p_f2_2(:, inlierSet);
    end
end

