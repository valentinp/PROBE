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
y_var = [5,5,5,5];
mu = zeros(4,1);
sigma = diag(y_var);

%Generate new landmarks
newLmNum = 40;

oldLmNum = 0;
oldLmPos = [];

% xRange = linspace(min(rho_i_pj_i(1,:)), max(rho_i_pj_i(1,:)), newLmNum);
% yRange = linspace(min(rho_i_pj_i(2,:)), max(rho_i_pj_i(2,:)), newLmNum);
% zRange = linspace(min(rho_i_pj_i(3,:)), max(rho_i_pj_i(3,:)), newLmNum);
% [newLmPosX, newLmPosY, newLmPosZ ] = meshgrid(xRange, yRange, zRange);

xRange = [min(rho_i_pj_i(1,:)) - 5, max(rho_i_pj_i(1,:)) + 5];
yRange = [min(rho_i_pj_i(2,:)) - 5, max(rho_i_pj_i(2,:)) + 5];
zRange = [min(rho_i_pj_i(3,:)) - 5, max(rho_i_pj_i(3,:)) ];

newLmPosX = range(xRange)*rand(1,newLmNum - oldLmNum) + xRange(1);
newLmPosY = range(yRange)*rand(1,newLmNum - oldLmNum) + yRange(1);
newLmPosZ = range(zRange)*rand(1,newLmNum - oldLmNum) + zRange(1);

newLmPos = [newLmPosX(:)'; newLmPosY(:)'; newLmPosZ(:)'];
newLmPos = [oldLmPos, newLmPos];

rho_i_pj_i = newLmPos;
T_cv = [C_c_v -C_c_v*rho_v_c_v; 0 0 0 1];
y_k_j = -1*ones(4, length(t), newLmNum);


for k = 1:length(t)
    
    %Add noise to the rotational velocity measurements
    %w_vk_vk_i(:,k) = w_vk_vk_i(:,k) + 0.1*randn(3,1);
    v_vk_vk_i(:,k) = v_vk_vk_i(:,k) + 0.1*randn(3,1);
    
    C_vi = axisAngleToRotMat(theta_vk_i(:,k));
    T_vi = [C_vi -C_vi*r_i_vk_i(:,k); 0 0 0 1];
    T_ci = T_cv*T_vi;
    
    %Add observations
    for lm_i = 1:size(newLmPos, 2)
        p_li_i = newLmPos(:,lm_i);
        p_lc_c = homo2cart(T_ci*cart2homo(p_li_i));
        [yMeas] = stereoCamProject(p_lc_c, calibParams);
        if all(yMeas > 0) && all(yMeas([1,3]) <= 640) && all(yMeas([2,4]) <= 480)
                if yMeas(1) > 250 %&& norm(w_vk_vk_i(:, k)) > 0.75
                    noise = mvnrnd(mu, 10*sigma)';
                else
                    noise = mvnrnd(mu, sigma)';
                end
                y_k_j(:,k, lm_i) = yMeas + noise;
        end
    end
end


save(['../datasets/dataset3_fresh_',num2str(newLmNum),'select.mat']);