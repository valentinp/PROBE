function [predWeights] = getPredVectorWeight(predVectors, searchObject, learnedWeights, refWeight, learnedParams)
%Use KNN to find nearest vector and associated weight
idx = knnsearch(searchObject, predVectors', 'K', learnedParams.k);
predWeights = mean(learnedWeights(idx), 2)/refWeight;
predWeights = predWeights.^learnedParams.gamma;
end