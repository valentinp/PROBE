kStart = 1;
kEnd = size(data.y_k_j,2)-1; 

firstState.C_vi = eye(3);
firstState.r_vi_i = zeros(3,1);


R = diag(100*ones(4,1));

optParams.RANSACCostThresh = 0.02;
optParams.maxGNIter = 10;
optParams.lineLambda = 0.5;
optParams.LMlambda = 1e-3;

%Misc
data.r_i_vk_i = data.p_vi_i;
optParams.minFeaturesForOpt = 5;