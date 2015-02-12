% clear all;
%addpath('../utils');
load('../datasets/dataset3.mat')

rng(42);

%Set up appropriate structs
calibParams.c_u = cu;
calibParams.c_v = cv;
calibParams.f_u = fu;
calibParams.f_v = fv;
calibParams.b = b;

%Noise parameters
mu = zeros(4,1);
sigma = diag(y_var);

T_cv = [C_c_v -C_c_v*rho_v_c_v; 0 0 0 1];
errorHist = [];

for k = 1:length(t)
    
    
    C_vi = axisAngleToRotMat(theta_vk_i(:,k));
    T_vi = [C_vi -C_vi*r_i_vk_i(:,k); 0 0 0 1];
    T_ci = T_cv*T_vi;
    
    %Add observations
    for lm_i = 1:size(rho_i_pj_i, 2)
        p_li_i = rho_i_pj_i(:,lm_i);
        p_lc_c = homo2cart(T_ci*cart2homo(p_li_i));
        [yMeas] = stereoCamProject(p_lc_c, calibParams);
        if y_k_j(:,k, lm_i) > -1 & norm(w_vk_vk_i(:, k)) < 0.5
                errorHist(:,end+1) = yMeas - y_k_j(:,k, lm_i);
        end
    end
end

%%
close all
subplot(2,2,1)
hist(errorHist(1,:), 40)
std(errorHist(1,:))
mean(errorHist(1,:))
subplot(2,2,2)
hist(errorHist(2,:), 40)
std(errorHist(2,:))
mean(errorHist(2,:))
subplot(2,2,3)
hist(errorHist(3,:), 40)
std(errorHist(3,:))
mean(errorHist(3,:))
subplot(2,2,4)
std(errorHist(4,:))
mean(errorHist(4,:))
hist(errorHist(4,:), 40)

%%
plot(sqrt(sum(v_vk_vk_i.^2,1)))

