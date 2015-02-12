function [rho_fc_c_all, validLmObsId] = triangulateAllPointsNoCheck(y_k, calibParams)
%TRIANGULATEALLPOINTS Triangulates all points in the camera frame

validLmObsId = 1:length(y_k(1, 1, :));
rho_fc_c_all = NaN(3, length(validLmObsId));

lm_i = 1;
for lmId = validLmObsId
       yMeas = y_k(:, 1, lmId);
       rho_fc_c_all(:, lm_i) = triangulate(yMeas, calibParams);
       lm_i = lm_i + 1;
end


end

