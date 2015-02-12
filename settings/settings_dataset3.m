kStart = 1200;
kEnd = 1700; 

R = diag(data.y_var);

firstState.C_vi = Cfrompsi(data.theta_vk_i(:,kStart));
firstState.r_vi_i = data.r_i_vk_i(:,kStart);

optParams.RANSACCostThresh = 0.05;
optParams.maxGNIter = 20;
optParams.lineLambda = 1;
optParams.LMlambda = 0.05;

optParams.minFeaturesForOpt = 3;