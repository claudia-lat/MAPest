function [ orderedData ] = matchOrderToTraversal( originalOrder, originalData, finalOrder )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

orderedData = zeros(size(originalData)); 
for i = 1 : length(finalOrder)
    index = indexOfString (originalOrder, finalOrder{i});
    orderedData(i,:) = originalData(index,:);
end
end

function [ index ] = indexOfString ( cellArray, stringName)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
  index = -1;
  for i = 1 : length(cellArray)
      if strcmp (cellArray{i}, stringName)
          index = i;
          break;
      end
  end
end