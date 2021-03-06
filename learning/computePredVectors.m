function [predVectors] = computePredVectors( pixelLocationsCurr, imageCurr, imagePrev, imuData)
%EXTRACTPREDICTIONSPACE Extract prediction vectors

%Predictors:
%[imuPred; blurMetricNum; localEntropy; lowFreqCoeff; highFreqCoeff; flowCoeff]; 
predSpaceDim = 7; 
predVectors = zeros(predSpaceDim, size(pixelLocationsCurr,2));

% matching parameters
param.nms_n                  = 2;   % non-max-suppression: min. distance between maxima (in pixels)
% init matcher
matcherMexFlow('init',param);
% push back images
matcherMexFlow('push',imageCurr);
matcherMexFlow('push',imagePrev);
matcherMexFlow('match',0);
p_matched_global = matcherMex('get_matches',0);
% close matcher
matcherMexFlow('close');
U = p_matched_global(3,:) - p_matched_global(1,:);
V = p_matched_global(4,:) - p_matched_global(2,:);
Uxcoord = p_matched_global(3,:);
Vxcoord = p_matched_global(4,:);


imageSize = size(imageCurr);
binSizeSmall = 100;
binSizeLarge = 400;

%Blur metric is the same for entire image
for i = 1:size(pixelLocationsCurr, 2)
    pixelLoc = round(pixelLocationsCurr(:,i));
    pixelLoc = flipud(pixelLoc); %Necessary so u and v are correct
    %Magnitude of linear acceleration and angular velocity
    imuPred = [norm(imuData(1:3))/9.8; norm(imuData(4:6))];
    
    %Ensure the location is not at the edge of the image, otherwise return
    %zeros for everything but pixel location
    
    if pixelLoc(1) < imageSize(1) - binSizeSmall/2 && pixelLoc(1)  > binSizeSmall/2 && pixelLoc(2) < imageSize(2) - binSizeSmall/2  && pixelLoc(2) > binSizeSmall/2
        
             
        imageSeg = imageCurr(pixelLoc(1)-binSizeSmall/2:pixelLoc(1)+binSizeSmall/2, pixelLoc(2)-binSizeSmall/2:pixelLoc(2)+binSizeSmall/2);
        
        blurMetricNum = blurMetric(imageSeg);                
        localEntropy = entropy(imageSeg);
        Ig = fft2(double(imageSeg));
        Iglf = Ig(1:binSizeSmall/5,1:binSizeSmall/5);
        Ighf = Ig(end-binSizeSmall/5:end,end-binSizeSmall/5:end);
        lowFreqCoeff = mean(log(abs(Iglf(:))));
        highFreqCoeff = mean(log(abs(Ighf(:))));
        
        maskSmall = Uxcoord > pixelLoc(2)-binSizeSmall/2 & Uxcoord < pixelLoc(2)+binSizeSmall/2 & Vxcoord > pixelLoc(1)-binSizeSmall/2 & Vxcoord < pixelLoc(1)+binSizeSmall/2;
        maskLarge = Uxcoord > pixelLoc(2)-binSizeLarge/2 & Uxcoord < pixelLoc(2)+binSizeLarge/2 & Vxcoord > pixelLoc(1)-binSizeLarge/2 & Vxcoord < pixelLoc(1)+binSizeLarge/2;

        UlocSmall = U(maskSmall);
        VlocSmall = V(maskSmall);

        UlocLarge = U(maskLarge);
        VlocLarge = V(maskLarge);

        if ~isempty(UlocSmall)
                meanNormSmall = mean(var([UlocSmall; VlocSmall]'));
                meanNormLarge = mean(var([UlocLarge; VlocLarge]'));
                flowCoeff = log(meanNormSmall/meanNormLarge);
                if flowCoeff < -20
                    flowCoeff = 0;
                end
        else
            flowCoeff = 0;
        end
   
        %local mean intensity
    else
        blurMetricNum = 0;
        localEntropy = 0;
        lowFreqCoeff = 0;
        highFreqCoeff = 0;
        flowCoeff = 0;
    end

    predVec = [imuPred; blurMetricNum; localEntropy; lowFreqCoeff; highFreqCoeff; flowCoeff];
    predVectors(:,i) = predVec;
    
    
end

end

