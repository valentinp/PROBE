function [predWeights] = getPredVectorWeight(predVectors, searchObject, learnedWeights, refWeight, gamma)
%Use KNN to find nearest vector and associated weight
idx = knnsearch(searchObject, predVectors', 'K', 50);
predWeights = mean(learnedWeights(idx), 2)/refWeight;
predWeights = predWeights.^gamma;
end