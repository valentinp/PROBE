function [predWeight] = getPredVectorWeight(predVector, searchObject, predWeights, refWeight)
%Use KNN to find nearest vector and associated weight
idx = knnsearch(searchObject, predVector', 'K', 25);
predWeight = mean(predWeights(idx))/refWeight;
predWeight = predWeight^35;
end