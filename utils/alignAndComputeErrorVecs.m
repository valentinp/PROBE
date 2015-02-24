function [errorVecs, VIOpts_aligned, T_gi] = ...
            alignAndComputeErrorVecs(GPSpts, VIOpts, k1, k2)       
%
% Aligns VIO points onto synchronized GPS points
%
% INPUT:    GPSpts, VIOpts -- 3xN array of points
%           k1, k2 -- (optional) start and end indices for the initial alignment
% OUTPUT:   errorVecs -- 3xN array of error vectors
%           VIOpts_aligned -- VIOpts transformed into the GPS frame
%           T_gi -- VIO frame to GPS frame transformation matrix
% 
    % Default kRange
    if nargin == 4
        kRange = k1:k2;
    else
        kRange = 1:20;
    end

    % Align VIO data in kRange to GPS data in kRange
    T_gi = scalarWeightedPointCloudAlignment(VIOpts(:,kRange), GPSpts(:,kRange));

    % Align the remaining VIO data
    VIOpts_aligned = homo2cart(T_gi * cart2homo(VIOpts));

    % Compute error vectors
    errorVecs = VIOpts_aligned - GPSpts;

end