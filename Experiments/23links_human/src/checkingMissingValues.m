function [missingIdx, flag] = checkingMissingValues(file,vectorToBeCompared, nrOfFrames, nrOfVariableRepetition, variableName)

missingIdx = [ ];
for nrOfFramesIdx = 1 : nrOfFrames
    % find the indices where the variable is missing
    if ~strcmp(file{vectorToBeCompared(nrOfFramesIdx)+nrOfVariableRepetition,1},variableName)
        missingIdx = [missingIdx,vectorToBeCompared(nrOfFramesIdx)+nrOfVariableRepetition];
        flag = true;
    end
end
end

