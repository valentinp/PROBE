close all;
load('2011_09_26_drive_0005_sync_learnedPredSpaceIter10.mat')
%scatter(learnedPredSpace.weights, zeros(1, length(learnedPredSpace.weights)))

%%
close all;
for i = 1:size(learnedPredSpace.predVectors,1)
    A = [learnedPredSpace.weights' ones(length(learnedPredSpace.weights),1)];
    x = (A'*A)\(A'*learnedPredSpace.predVectors(i,:)');
    x(1)
    figure
    plot(learnedPredSpace.weights, learnedPredSpace.predVectors(i,:), '*')
    hold on
    uniqueWeights = unique(learnedPredSpace.weights);
    meanPredictors = NaN(1, length(uniqueWeights));
    for w = 1:length(uniqueWeights)
        meanPredictors(w) = median(learnedPredSpace.predVectors(i,learnedPredSpace.weights==uniqueWeights(w)));
    end
    plot(uniqueWeights, meanPredictors, 'r*');
    xlabel('ARMSE')
    ylabel('Predictor')
end


% subplot(5,2,7)
% subplot(5,2,8)
% plot(learnedPredSpace.weights, learnedPredSpace.predVectors(8,:), '*')
% subplot(5,2,9)
% plot(learnedPredSpace.weights, learnedPredSpace.predVectors(9,:), '*')

