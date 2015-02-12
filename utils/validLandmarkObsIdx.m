function [ validIdx ] = validLandmarkObsIdx( pixelMeas )
%VALIDLANDMARKOBSIDX Returns indices where valid measurements are found
validIdx = find(pixelMeas > -1);
end

