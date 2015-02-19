function [predWeights] = getPredVectorWeight(predVectors, searchObject, learnedWeights, refWeight)
%Use KNN to find nearest vector and associated weight
idx = knnsearch(searchObject, predVectors', 'K', 25);
predWeights = mean(learnedWeights(idx), 2)/refWeight;
predWeights = predWeights.^8;
end