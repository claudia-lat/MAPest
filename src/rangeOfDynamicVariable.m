function [ index, len ] = rangeOfDynamicVariable( berdy, varType, varID)
%RANGEOFDYNAMICVARIABLE Summary of this function goes here
%   Detailed explanation goes here

dynVariable = berdy.getDynamicVariablesOrdering();
index = -1;
len = 0;
for i = 1:size(dynVariable,2)
    currentInfo = dynVariable{i};
    if currentInfo.type == varType && strcmp(currentInfo.id, varID)
        range = currentInfo.range;
        index = range.offset + 1;
        len = range.size;
        break;
    end
end
end