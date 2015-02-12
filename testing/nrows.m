function [nr] = nrows(A)
% NROWS Number of rows in an array.
%
%   [nr] = NROWS(A) returns the number of rows in the array/matrix A.
%
%   Inputs:
%   -------
%    A  - mxn array/matrix.
%
%   Outputs:
%   --------
%    nr - Number of rows in A (i.e. m).

nr = size(A, 1);