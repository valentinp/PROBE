function [p_f1_1, p_f2_2] = triangulateAllPointsDirect(y_k, calibParams)
%TRIANGULATEALLPOINTS y_k: 8xN pairs of stereo points

numLm = size(y_k,2);
p_f1_1 = NaN(3, numLm);
p_f2_2 = NaN(3, numLm);


lm_i = 1;
for lmId = 1:numLm
       yMeas1 = y_k(1:4, lmId);
       yMeas2 = y_k(5:8, lmId);
       p_f1_1(:, lm_i) = triangulate(yMeas1, calibParams);
       p_f2_2(:, lm_i) = triangulate(yMeas2, calibParams);
       lm_i = lm_i + 1;
end

end

