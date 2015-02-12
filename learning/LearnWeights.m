clc
learnedPredSpace.predVectors = [];
learnedPredSpace.weights = [];

for run_i = 1:5

tic
[swfStats, observedPredVectors] = SlidingWindowTrain();
toc
swfStats.rmse_trans
swfStats.rmse_rot
rmse_metric = (swfStats.rmse_trans  + swfStats.rmse_rot)/2

%Update the prediction space learning
%subsetWeight = 4.685/(1+rmse_metric^2)^2; %4.685 = 95% Asymptotic Efficiency Constat
subsetWeight = rmse_metric;
learnedPredSpace.predVectors = [learnedPredSpace.predVectors observedPredVectors];
learnedPredSpace.weights = [learnedPredSpace.weights subsetWeight*ones(1, size(observedPredVectors,2))];
end

learnedPredSpace.weights = max(min(learnedPredSpace.weights)./learnedPredSpace.weights,0);

%%
useWeights = true;
learnedPredSpace.predVectors = [];
[swfStats] = SlidingWindowTest(learnedPredSpace, useWeights)
norm(swfStats.swf_trans_err(:, end))
norm(swfStats.swf_rot_err(:, end))
%%
worstIdx = learnedPredSpace.weights > 0.99;
close all
figure
plot(learnedPredSpace.predVectors(1,worstIdx), 'r*')
%scatter(learnedPredSpace.predVectors(1,worstIdx), learnedPredSpace.predVectors(2,worstIdx), '*r')
hold on
%scatter(learnedPredSpace.predVectors(1,~worstIdx), learnedPredSpace.predVectors(2,~worstIdx), '*b')

 %%
% data = load('datasets/dataset3_fresh_40select.mat');
% plot(sqrt(sum(data.w_vk_vk_i.^2,1)))