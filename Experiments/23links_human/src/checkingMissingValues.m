function [missingIdx, flag] = checkingMissingValues(file,IWearRemapperIndx, nrOfFrames, nrOfVariableRepetition, variableName)

missingIdx = [ ];
for nrOfFramesIdx = 1 : nrOfFrames
    % find the indices where the variable is missing
    if ~strcmp(file{IWearRemapperIndx(nrOfFramesIdx)+nrOfVariableRepetition,1},variableName)
        missingIdx = [missingIdx,IWearRemapperIndx(nrOfFramesIdx)+nrOfVariableRepetition];
        flag = true;
    end
end
end

