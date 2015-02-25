% demonstrates sparse optical flow
disp('===========================');
clear; close all;
addpath('../libviso2');
addpath('../learning');
addpath('../datasets/extraction/utils');
addpath('../datasets/extraction/utils/devkit');
addpath('../utils');
% matching parameters
param.nms_n                  = 5;   % non-max-suppression: min. distance between maxima (in pixels)
param.nms_tau                = 50;  % non-max-suppression: interest point peakiness threshold
param.match_binsize          = 10;  % matching bin width/height (affects efficiency only)
param.match_radius           = 200; % matching radius (du/dv in pixels)
param.match_disp_tolerance   = 1;   % du tolerance for stereo matches (in pixels)
param.outlier_disp_tolerance = 5;   % outlier removal: disparity tolerance (in pixels)
param.outlier_flow_tolerance = 1;   % outlier removal: flow tolerance (in pixels)
param.multi_stage            = 1;   % 0=disabled,1=multistage matching (denser and faster)
param.half_resolution        = 0;   % 0=disabled,1=match at half resolution, refine at full resolution
param.refinement             = 0;   % refinement (0=none,1=pixel,2=subpixel)

dataBaseDir = '/Users/valentinp/Desktop/KITTI/2011_09_26/2011_09_26_drive_0022_sync';
dataCalibDir = '/Users/valentinp/Desktop/KITTI/2011_09_26';
% Get ground truth and import data
frameRange = 21:23;
%Image data

leftImageData = loadImageData([dataBaseDir '/image_00'], frameRange);
%%

main1 = uint8(leftImageData.rectImages(:,:,1));
figure
for frame = frameRange(3:end)
    
main2 = uint8(leftImageData.rectImages(:,:,frame-frameRange(1)));


param.nms_n = 8;

% init matcher
matcherMex('init',param);

% push back images
matcherMex('push',main1);
matcherMex('push',main2);
matcherMex('match',0);
p_matched_sparse = matcherMex('get_matches',0);
% close matcher
matcherMex('close');

% init matcher
param.nms_n = 5;
matcherMex('init',param);

% push back images
matcherMex('push',main1);
matcherMex('push',main2);
matcherMex('match',0);
p_matched_dense = matcherMex('get_matches',0);
% close matcher
matcherMex('close');

U = p_matched_dense(3,:) - p_matched_dense(1,:);
V = p_matched_dense(4,:) - p_matched_dense(2,:);
    
cla;
imshow(main2);
title(sprintf('%d',frame));

%plotMatch(main2,p_matched_dense,0);
hold on;

binSizeSmall = 100;
binSizeLarge = 400;

imageCurr = main2;
Uxcoord = p_matched_dense(3,:);
Vxcoord = p_matched_dense(4,:);

imageSize = size(imageCurr);

coeffList = [];
coeffLocList = [];
for f_i = 1:size(p_matched_sparse, 2)
     
    pixelLoc = p_matched_sparse(3:4,f_i);
    pixelLoc = flipud(pixelLoc);
        if pixelLoc(1) < imageSize(1) - binSizeSmall/2 && pixelLoc(1)  > binSizeSmall/2 && pixelLoc(2) < imageSize(2) - binSizeSmall/2  && pixelLoc(2) > binSizeSmall/2

        imageSeg = imageCurr(pixelLoc(1)-binSizeSmall/2:pixelLoc(1)+binSizeSmall/2, pixelLoc(2)-binSizeSmall/2:pixelLoc(2)+binSizeSmall/2);
    %imageSegLarge = imageCurr(pixelLoc(1)-binSizeLarge/2:pixelLoc(1)+binSizeLarge/2, pixelLoc(2)-binSizeLarge/2:pixelLoc(2)+binSizeLarge/2);
    
%         blurMetricNum = blurMetric(imageSeg);                
%         localEntropy = entropy(imageSeg);
         Ig = fft2(double(imageSeg));
         Iglf = Ig(1:binSizeSmall/5,1:binSizeSmall/5);
         Ighf = Ig(end-binSizeSmall/5:end,end-binSizeSmall/5:end);
         lowFreqCoeff = mean(log(abs(Iglf(:))));
         highFreqCoeff = mean(log(abs(Ighf(:))));

         coeffLocList(:,end+1) = pixelLoc;
         coeffList(end+1) = highFreqCoeff;
    
        end
    %viscircles([pixelLoc(2), pixelLoc(1)], radius, 'EdgeColor', [red green 0], 'DrawBackgroundCircle', true);
    %text(u+binSize/2,v+binSize/2+10,sprintf('%.1f',Ighf), 'Color', 'g');
    
end

for c_i = 1:length(coeffList)
    radius = 10;
    freqCoeff = coeffList(c_i) - mean(coeffList);
    pixelLoc = coeffLocList(:,c_i);
    
    red = 1/(1 + exp(10*freqCoeff));
    green = 1/(1 + exp(-10*freqCoeff));
    h = rectangle('Position',[pixelLoc(2),pixelLoc(1),radius,radius],...
    'Curvature',[1,1],...
    'FaceColor',[red green 0]);
    text(pixelLoc(2),pixelLoc(1),sprintf('%.1f',freqCoeff), 'Color', 'c');
end

main1 = main2;
drawnow();
end



% 
% divergence(X,Y,U,V)
%%
%%# Read an image
I = main1;
Ig = fft2(double(I));
Iglf = Ig(1:50,1:50);
Ighf = Ig(end-50:end,end-50:end);
mean(log(abs(Iglf(:))))
mean(log(abs(Ighf(:))))

