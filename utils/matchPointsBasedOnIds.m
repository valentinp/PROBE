function [p_f1_1, p_f2_2] = matchPointsBasedOnIds(oldPoints, oldPointIds, newPoints, newPointIds)
%MATCHPOINTSBASEDONIDS Match point clouds based on point IDS

mutualIds = intersect(oldPointIds, newPointIds);

[~, matchedOldIds] = ismember(mutualIds,oldPointIds);
[~, matchedNewIds] = ismember(mutualIds,newPointIds);


if ~isempty(matchedNewIds)
    p_f1_1 = oldPoints(:, matchedOldIds);
    p_f2_2 = newPoints(:, matchedNewIds);
else
    p_f1_1 = [];
    p_f2_2 = [];
end

end

