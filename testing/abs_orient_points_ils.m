function [Hif, resid] = abs_orient_points_ils(Pi, Pf, Si, Sf, iters, R0)
% ABS_ORIENT_POINTS_ILS Estimate absolute orientation from correspondences.
%
%   [Hif, resid] = ABS_ORIENT_POINTS_ILS(Pi, Pf, Si, Sf, iters, R0) 
%   estimates the 6-DOF change in pose of a sensor, given a series of 3D  
%   point correspondences.  Arrays Pi and Pf store corresponding landmark 
%   points before and after a change in pose.  Covariance matrices for the
%   points are stored in Si and Sf.
%
%   An optional initial guess, R0, for the rotation may be supplied.  The 
%   computed estimate is a maximum-likelihood estimate, given a Gaussian
%   noise assumption.
%
%   Inputs:
%   -------
%    Pi     - 3xn array of points (initial).
%    Pf     - 3xn array of points (final).
%    Si     - 3x3xn array of covariance matrices.
%    Sf     - 3x3xn array of covariance matrices.
%    iters  - Number of least-squares iterations.
%   [R0]    - Initial estimate for 3x3 rotation matrix.
%
%   Outputs:
%   --------
%    Hif     - 4x4 homogeneous transform matrix, frame 'f' in frame 'i'.
%   [resid]  - Residual sum-of-squared error for estimate.

% Find initial rotation matrix.
if nargin == 6
  R = R0;
else
  % Initial estimate using standard least squares.
  Hif = abs_orient_points_ls(Pi, Pf, Si, Sf);
  R = Hif(1:3, 1:3);
end

% Extract angles from R.
rpy = rpy_from_dcm(R);
I = eye(3);

% Iterate.
for j = 1 : iters,
  A = zeros(6, 6);
  B = zeros(6, 1);

  % Build least-squares matrix.
  for i = 1 : ncols(Pi),
    [dRdr, dRdp, dRdy] = dcm_jacob_rpy(rpy);
    Ji = [dRdr*Pi(:, i), dRdp*Pi(:, i), dRdy*Pi(:, i)];

    Hi = [Ji, I];
    Wi = inv(Sf(:, :, i) + R*Si(:, :, i)*R');
    Li = Pf(:, i) - R*Pi(:, i) + Ji*rpy;
    
    A = A + Hi'*Wi*Hi;
    B = B + Hi'*Wi*Li;
  end

  % Solve system.
  theta = A\B;

  % Solve system, then update R and iterate.
  rpy = theta(1:3);
  R = dcm_from_rpy(rpy);
  T = theta(4:6);
end

Hif = [R, T; 0, 0, 0, 1];

% Compute RMS residual, if desired.  TODO fix THISSSS!!
if nargout == 2
  resid = (Pf - R*Pi - repmat(T, [1, ncols(Pi)])).^2;
  resid = sum(resid);
end