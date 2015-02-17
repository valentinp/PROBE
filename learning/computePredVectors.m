function [predVectors] = computePredVectors( pixelLocations, image, imuData)
%EXTRACTPREDICTIONSPACE Extract prediction vectors

%Pre-allocate space for prediction vectors


usePredDims = [1; ... %u
               0; ... %v
               0; ... %x contrast
               0; ... %y contrast
               0; ... % entropy
               0; ... % mag(accel)
               0; ... % mag(omega)
               0; ... % point density
               0;     % blur metric
               ];
predSpaceDim = sum(usePredDims); 
predVectors = zeros(predSpaceDim, size(pixelLocations,2));


%normalize rgb image
imageSize = size(image(:,:,1))';

%Blur metric is the same for entire image
if usePredDims(9)
    blurMetricNum = blurMetric(image);                
end    

for i = 1:size(pixelLocations, 2)
    pixelLoc = round(pixelLocations(:,i));
    pixelLoc = flipud(pixelLoc);
    predVec = [];
    %Magnitude of linear acceleration and angular velocity
    imuPred = [norm(imuData(1:3))/10; norm(imuData(4:6))/0.2];
    
    %Ensure the location is not at the edge of the image, otherwise return
    %zeros for everything but pixel location
    
    if pixelLoc(1) < size(image, 1) && pixelLoc(1)  > 1 && pixelLoc(2) < size(image,2)  && pixelLoc(2) > 1
        
             
        %local contrast vector
        grayImgSeg = (image(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, :));
        
        localContrast = [grayImgSeg(1, 2) - grayImgSeg(3, 2); ...
            grayImgSeg(2, 1) - grayImgSeg(2, 3);
        ];
        
        %local mean intensity
        localEntropy = entropy(grayImgSeg);
    else
        localContrast = zeros(2,1);
        localEntropy = 0;
    end
    
    %Landmark Density radius 
    lmSearchRadius = 5;
    %Now add pixel density if we are at least 5 pixels away from the edge
    lmDensity = sum( pixelLocations(1,:) < pixelLoc(1) + lmSearchRadius & ...
                     pixelLocations(1,:) > pixelLoc(1) - lmSearchRadius & ...
                     pixelLocations(2,:) < pixelLoc(2) + lmSearchRadius & ...
                        pixelLocations(2,:) > pixelLoc(2) - lmSearchRadius);
                    
    
   
        if usePredDims(1)
            predVec = [predVec; pixelLoc(1)/imageSize(1)];
        end
        if usePredDims(2)
            predVec = [predVec; pixelLoc(2)/imageSize(2)];
        end
        if usePredDims(3)
            predVec = [predVec; localContrast(1)];
        end
        if usePredDims(4)
            predVec = [predVec; localContrast(2)];
        end
        if usePredDims(5)
            predVec = [predVec; localEntropy];
        end
        if usePredDims(6)
            predVec = [predVec; imuPred(1)];
        end
        if usePredDims(7)
            predVec = [predVec; imuPred(2)];
        end
        if usePredDims(8)
            predVec = [predVec; lmDensity];
        end
        if usePredDims(9)
            predVec = [predVec; blurMetricNum];
        end
        
        predVectors(:,i) = predVec;
    
    
end

end

