function idx = evalWeight(yMeas, imuMeasurement, knnSpaceObject)
%EVALWEIGHT Evaluates the weight based on k-nearest neightbour approach

predVector = [yMeas(1:2); norm(imuMeasurement.v); norm(imuMeasurement.omega)];
idx = knnsearch(knnSpaceObject, predVector', 'K', 5);

end

