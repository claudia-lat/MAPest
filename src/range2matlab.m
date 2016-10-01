function [ matlabRange ] = range2matlab( iDynTreeRange )
% RANGE2MATLAB Convert an iDynTree.Range struct to a Matlab range.

matlabRange = (iDynTreeRange.offset+1):(iDynTreeRange.offset+iDynTreeRange.size);
end

