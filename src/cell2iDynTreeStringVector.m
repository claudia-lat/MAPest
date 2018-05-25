function [ selectedJoints ] = cell2iDynTreeStringVector( array )
% CELL2IDYNTREESTRINGVECTOR transforms an iDynTree StringVector object into
% cella array.

selectedJoints = iDynTree.StringVector();
for i = 1 : length(array)
    selectedJoints.push_back(array{i});
end
assert(selectedJoints.size == length(array), 'Vector size mismatch');
end
