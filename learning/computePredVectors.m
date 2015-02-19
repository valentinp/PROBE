function [predVectors] = computePredVectors( pixelLocationsCurr, imageCurr, imagePrev, imuData)
%EXTRACTPREDICTIONSPACE Extract prediction vectors

%Predictors:
%[imuPred; blurMetricNum; localEntropy; lowFreqCoeff; highFreqCoeff; flowCoeff]; 
predSpaceDim = 7; 
predVectors = zeros(predSpaceDim, size(pixelLocationsCurr,2));

% matching parameters
param.nms_n                  = 2;   % non-max-suppression: min. distance between maxima (in pixels)
param.nms_tau                = 50;  % non-max-suppression: interest point peakiness threshold
param.match_binsize          = 10;  % matching bin width/height (affects efficiency only)
param.match_radius           = 10; % matching radius (du/dv in pixels)
param.match_disp_tolerance   = 1;   % du tolerance for stereo matches (in pixels)
param.outlier_disp_tolerance = 5;   % outlier removal: disparity tolerance (in pixels)
param.outlier_flow_tolerance = 5;   % outlier removal: flow tolerance (in pixels)
param.multi_stage            = 1;   % 0=disabled,1=multistage matching (denser and faster)
param.half_resolution        = 0;   % 0=disabled,1=match at half resolution, refine at full resolution
param.refinement             = 0;   % refinement (0=none,1=pixel,2=subpixel)


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

normFlowMeanGlobal = mean(sqrt(sum([U.^2; V.^2])));


imageSize = size(imageCurr);
binSize = 50;

%Blur metric is the same for entire image
for i = 1:size(pixelLocationsCurr, 2)
    pixelLoc = round(pixelLocationsCurr(:,i));
    pixelLoc = flipud(pixelLoc); %Necessary so u and v are correct
    %Magnitude of linear acceleration and angular velocity
    imuPred = [norm(imuData(1:3))/9.8; norm(imuData(4:6))];
    
    %Ensure the location is not at the edge of the image, otherwise return
    %zeros for everything but pixel location
    
    if pixelLoc(1) < imageSize(1) - binSize/2 && pixelLoc(1)  > binSize/2 && pixelLoc(2) < imageSize(2) - binSize/2  && pixelLoc(2) > binSize/2
        
             
        imageSeg = imageCurr(pixelLoc(1)-binSize/2:pixelLoc(1)+binSize/2, pixelLoc(2)-binSize/2:pixelLoc(2)+binSize/2);
        
        blurMetricNum = blurMetric(imageSeg);                
        localEntropy = entropy(imageSeg);
        Ig = fft2(double(imageSeg));
        Iglf = Ig(1:binSize/5,1:binSize/5);
        Ighf = Ig(end-binSize/5:end,end-binSize/5:end);
        lowFreqCoeff = mean(log(abs(Iglf(:))));
        highFreqCoeff = mean(log(abs(Ighf(:))));
        
        mask = Uxcoord > pixelLoc(2)-binSize/2 & Uxcoord < pixelLoc(2)+binSize/2 & Vxcoord > pixelLoc(1)-binSize/2 & Vxcoord < pixelLoc(1)+binSize/2;
        Uloc = U(mask);
        Vloc = V(mask);
        flowCoeff = mean(sqrt(sum([Uloc.^2; Vloc.^2])))/normFlowMeanGlobal;
        
        if isnan(flowCoeff)
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

