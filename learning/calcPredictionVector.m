function  predVector = calcPredictionVector(yMeas, imuMeasurement)
%TRACKPREDICITONVECTORS Appends prediction space vector
%predVector = [yMeas(1:2); norm(imuMeasurement.v); norm(imuMeasurement.omega)];
predVector = [norm(imuMeasurement.omega)];
end

