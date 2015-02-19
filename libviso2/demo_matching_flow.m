% demonstrates sparse optical flow
disp('===========================');
clear all; dbstop error; close all;

% read images from file
 I1p = imread('/Users/valentinp/Desktop/KITTI/2011_09_26/2011_09_26_drive_0005_sync/image_00/data/0000000000.png');
 I1c = imread('/Users/valentinp/Desktop/KITTI/2011_09_26/2011_09_26_drive_0005_sync/image_00/data/0000000001.png');

%I1p = imread('/Volumes/STARSExFAT/KITTI/2011_09_26/2011_09_26_drive_0096_sync/image_00/data/0000000000.png');
% I1c = imread('/Volumes/STARSExFAT/KITTI/2011_09_26/2011_09_26_drive_0096_sync/image_00/data/0000000001.png');

 %I1p = I1p(47:296,224:436);
 %I1c = I1c(47:296,224:436);
 

% matching parameters
param.nms_n                  = 5;   % non-max-suppression: min. distance between maxima (in pixels)
param.nms_tau                = 50;  % non-max-suppression: interest point peakiness threshold
param.match_binsize          = 50;  % matching bin width/height (affects efficiency only)
param.match_radius           = 200; % matching radius (du/dv in pixels)
param.match_disp_tolerance   = 1;   % du tolerance for stereo matches (in pixels)
param.outlier_disp_tolerance = 5;   % outlier removal: disparity tolerance (in pixels)
param.outlier_flow_tolerance = 5;   % outlier removal: flow tolerance (in pixels)
param.multi_stage            = 1;   % 0=disabled,1=multistage matching (denser and faster)
param.half_resolution        = 0;   % 0=disabled,1=match at half resolution, refine at full resolution
param.refinement             = 2;   % refinement (0=none,1=pixel,2=subpixel)

% init matcher
matcherMex('init', param);

% push back images
matcherMex('push',I1p);
tic
matcherMex('push',I1c);
disp(['Feature detection: ' num2str(toc) ' seconds']);

% match images
tic; matcherMex('match',0);
p_matched = matcherMex('get_matches',0);
disp(['Feature matching:  ' num2str(toc) ' seconds']);

% close matcher
matcherMex('close');

% show matching results
disp(['Number of matched points: ' num2str(length(p_matched))]);
disp('Plotting ...');
plotMatch(I1c,p_matched,0);

%%
%Compute divergence
X = p_matched(1,:);
Y = p_matched(2,:);

[Xm, Ym] = meshgrid(X, Y);

%%
Um = NaN(length(X), length(X));
Vm = NaN(length(X), length(X));

U = p_matched(3,:) - p_matched(1,:);
V = p_matched(4,:) - p_matched(2,:);

Um(logical(eye(size(Um)))) = U;
Vm(logical(eye(size(Vm)))) = V;

% 
% if ~isempty(U)
%     predictor = var(sqrt(sum([U.^2; V.^2])))
% else
%     predictor = 0
% end

