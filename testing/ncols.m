function [nc] = ncols(A)
% NCOLS Number of columns in an array.
%
%   [nr] = NCOLS(A) returns the number of columns in the array/matrix A.
%
%   Inputs:
%   -------
%    A  - mxn array/matrix.
%
%   Outputs:
%   --------
%    nc - Number of columns in A (i.e. n).

nc = size(A, 2);