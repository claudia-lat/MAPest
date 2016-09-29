function [ selectedJoints ] = cell2iDynTreeStringVector( array )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

selectedJoints = iDynTree.StringVector();
for i = 1 : length(array)
     selectedJoints.push_back(array{i});
end 

assert(selectedJoints.size == length(array), 'Vector size mismatch');
end

