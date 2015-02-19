close all;
load('2011_09_26_drive_0005_sync_learnedPredSpaceIter10Step.mat')
%scatter(learnedPredSpace.weights, zeros(1, length(learnedPredSpace.weights)))

%%
% close all;
% pruneWeightsIdx = learnedPredSpace.predVectors(1,:) == 0 | learnedPredSpace.predVectors(2,:) == 0 | learnedPredSpace.predVectors(3,:) == 0 | learnedPredSpace.predVectors(4,:) == 0 | learnedPredSpace.predVectors(5,:) == 0 | learnedPredSpace.predVectors(6,:) == 0 | learnedPredSpace.predVectors(7,:) == 0;
% 
% learnedPredSpace.weights = learnedPredSpace.weights(~pruneWeightsIdx);
% learnedPredSpace.predVectors = learnedPredSpace.predVectors(:,~pruneWeightsIdx);
% 
% 
% for i = 1:size(learnedPredSpace.predVectors,1)
%     A = [learnedPredSpace.weights' ones(length(learnedPredSpace.weights),1)];
%     x = (A'*A)\(A'*learnedPredSpace.predVectors(i,:)');
%     x(1)
%     figure
%     plot(learnedPredSpace.weights, learnedPredSpace.predVectors(i,:), '*')
%     hold on
%     uniqueWeights = unique(learnedPredSpace.weights);
% %     meanPredictors = NaN(1, length(uniqueWeights));
% %     for w = 1:length(uniqueWeights)
% %         meanPredictors(w) = median(learnedPredSpace.predVectors(i,learnedPredSpace.weights==uniqueWeights(w)));
% %     end
% %     plot(uniqueWeights, meanPredictors, 'r*');
%     xlabel('ARMSE')
%     ylabel('Predictor')
% end


% subplot(5,2,7)
% subplot(5,2,8)
% plot(learnedPredSpace.weights, learnedPredSpace.predVectors(8,:), '*')
% subplot(5,2,9)
% plot(learnedPredSpace.weights, learnedPredSpace.predVectors(9,:), '*')

%%
%Perform 10 fold cross validation
kTestList = 25:100;

lw = length(learnedPredSpace.weights);
numBuckets = 10;
skip = floor(lw/numBuckets);
buckets = [1:skip:lw - mod(lw, numBuckets); skip:skip:lw];

kAvgErr = NaN(numBuckets, length(kTestList));
%
for b_i = 1:size(buckets,2)
    b_i
    trainIdx = [];
    testIdx = buckets(1,b_i):buckets(2,b_i);
    for btemp = 1:size(buckets,2)
        if btemp ~= b_i
            trainIdx =  [trainIdx buckets(1,btemp):buckets(2,btemp)];
        end
    end
    trainVectors = learnedPredSpace.predVectors(:, trainIdx)';
    trainWeights = learnedPredSpace.weights(trainIdx)';
    testVectors = learnedPredSpace.predVectors(:, testIdx)';
    testWeights = learnedPredSpace.weights(testIdx)';
    
    searchObject = KDTreeSearcher(trainVectors);
    
    for k_i = 1:length(kTestList)
        k = kTestList(k_i);
        idx = knnsearch(searchObject, testVectors, 'K', k);
        
    	predictedWeights = mean(trainWeights(idx),2);
        kAvgErr(b_i,k_i) = mean(abs(predictedWeights - testWeights));
    end
end
mean(kAvgErr, 1)
plot(mean(kAvgErr, 1))

