function [M, inliers] = ransac(data, sys, optargs)
% RANSAC Robustly fit a model to data using the RANSAC algorithm.
%
%   [M, inliers] = RANSAC(data, sys, optargs) fits a model to data robustly, 
%   by choosing random subsets of the data, fitting a model, and then checking 
%   the number of inliers.
%
%   The user must supply three functions.  The fitting function should have
%   the signature: 
%
%    M = sys.fittingFun(data, ind, optargs)
%
%   where ind is an array of indicies in data indicating which poitns to use
%   when fitting a model.  If this function cannot fit a model it should 
%   return M as an empty matrix.
%
%   The distance function, which evaluates the distances from the model to
%   data, should have the following signature: 
%
%    [inliers, M] = sys.distFun(M, subdata, distThresh)
%
%   This function must evaluate the distances between points and the model 
%   returning the indices of elements in data that are inliers, that is, the
%   points that are within the distance threshold of the model.  Additionally,
%   if M is a cell array of possible models 'distfn' will return the model
%   has the most inliers.  If there is only one model this function must still
%   copy the model to the output.  After this call M will be a non-cell object 
%   representing only one model.
%
%   The degenerate function, which determines if a set of data points
%   produce a degenerate model, should have the signature:
%
%    r = sys.degenFun(subdata)
%
%   It may be that you cannot devise a test for degeneracy in which case you
%   can omit the function and rely on 'fittingFun' to return an empty model 
%   if the data set is degenerate.
%
%   This code is based on an original implementation by Peter Kovesi 
%   (University of Western Australia).
%
%   Inputs:
%   -------
%    data                - dxn data set array.
%
%    sys                 - Struct - system description.
%    sys.fittingFun      - Function - fitting function. 
%    sys.distFun         - Function - distance function.
%   [sys.degenFun]       - Function - degenerate model function.
%    sys.sampleMin       - Minimum number of samples needed to fit a model.
%    sys.distThresh      - Distance threshold between a point and model, 
%                          used to decided if the point is an inlier.
%
%   [optargs]            - Struct - optional arguments.
%    optargs.p           - Desired probability of chosing outlier free sample
%                          (default: 0.95).
%    optargs.maxTrials   - Maximum number of trials before giving up
%                          (default: 500).
%    optargs.maxAttempt  - Maximum number of attempts to select
%                          non-degenerate set of data (default: 100).
%    optargs.verbose     - Flag - verbose output (default: false).
%
%   Outputs:
%   --------
%    M        - Model with the greatest number of inliers.
%    inliers  - dxm array of indices of the elements of data that were the
%               inliers for the best model.
%
% References:
%
%    M.A. Fishler and  R.C. Boles. "Random sample concensus: A paradigm
%    for model fitting with applications to image analysis and automated
%    cartography". Comm. Assoc. Comp, Mach., Vol 24, No 6, pp 381-395, 1981
%
%    Richard Hartley and Andrew Zisserman. "Multiple View Geometry in
%    Computer Vision". pp 101-113. Cambridge University Press, 2001

% Defaults.
p = 0.95;           % Desired probability of choosing at least one sample
                    % free from outliers
maxTrials  =  500;  % Max number of trials before giving up.
maxAttempt =  100;  % Max number of attempts to select a non-degenerate
                    % data set.
verbose = false;    % No output.

if nargin == 3
 if isfield(optargs, 'p')
   p = optargs.p;
 end
 if isfield(optargs, 'maxTrials')
   maxTrials = optargs.maxTrials;
 end
 if isfield(optargs, 'maxAttempt')
   maxAttempt = optargs.maxAttempt;
 end
 if isfield(optargs, 'verbose')
  verbose = optargs.verbose;
 end
end
    
[~, npts] = size(data);
                     
bestM = NaN;        % Sentinel value allowing detection of solution failure.
trialCount = 0;
bestScore =  0;
N = 1000000000;     % Dummy initialisation for number of trials.

while N > trialCount
  % Select at random s datapoints to form a trial model, M.  In selecting 
  % these points we may have to check that they are not in a degenerate 
  % configuration.
  degenerate = 1; count = 1;
  
  while degenerate
    % Generate s random indicies in the range 1..npts
    ind = randsample(npts, sys.sampleMin);
    
    % Test that these points are not a degenerate configuration.
    if isfield(sys, 'degenFun')
      degenerate = sys.degenFun(data(:, ind));
    else
      degenerate = 0;
    end
      
    if ~degenerate
      % Fit model to random selection of data points.
      M = sys.fittingFun(data, ind, optargs);
      
      % Depending on your problem it might be that the only way you
      % can determine whether a data set is degenerate or not is to
      % try to fit a model and see if it succeeds.  If it fails we
      % reset degenerate to true.
      if isempty(M)
        degenerate = 1;
      end
    end
    
    % Don't get stuck in loop forever.
    count = count + 1;
    
    if count > maxAttempt
      warning('RANSAC was unable to select a nondegenerate data set');
      break
    end
  end
        
  % Once we are out here we should have some kind of model...
  % Evaluate distances between points and model returning the indices
  % of elements data that are inliers.
  [inliers, M] = sys.distFun(M, data, sys.distThresh);
  
  % Find the number of inliers to this model.
  ninliers = length(inliers);
  
  if ninliers > bestScore    % Largest set of inliers so far...
    bestScore = ninliers;    % Record data for this model.
    bestInliers = inliers;
    bestM = M;
    
    % Update estimate of N, the number of trials to ensure we pick,
    % with probability p, a data set with no outliers.
    fracInliers =  ninliers/npts;
    pNoOutliers = 1 - fracInliers^sys.sampleMin;
    pNoOutliers = max(eps, pNoOutliers);    % Avoid division by -Inf
    pNoOutliers = min(1-eps, pNoOutliers);  % Avoid division by 0.
    N = log(1-p)/log(pNoOutliers);
  end
  
  trialCount = trialCount + 1;
  
  if verbose
    fprintf('RANSAC Trial %d out of %d\r', trialCount, ceil(N));
  end
  
  % Don't get stuck in loop forever.
  if trialCount > maxTrials
    warning('RANSAC reached the maximum number of %d trials.', maxTrials);
    break
  end
end

if verbose
  fprintf('\n');
end
  
if ~isnan(bestM)   % Got a solution.
  M = bestM;
  inliers = bestInliers;
else
  error('RANSAC was unable to find a useful solution.');
end