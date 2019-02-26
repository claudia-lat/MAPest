
%% Structure of the data
% Per each timestamp the humanState .log file provides data according to
% the following structure:

% STATUS: 1 value
% -----
% UNKNOWN: 1 value, if data are dumped with yarpdatadumper rx/tx time (discard it!)
% -----
% TIMPESTAMP: 1 value
% -----
% JOINTS LABELS: list of 48/66 joints depending on the type of the acquisition
% -----
% JOINTS ANGLE: list of 48/66 joints angle ordered as the previous list
% -----
% BASE LABEL: label of the base
% -----
% BASE QUATERNION: 4 values
% -----
% BASE POSITION: 3 values
% -----
% BASE 6D VELOCITY: 6 values (3 linear, 3 angular)

% TODO: base quantities expressed w.r.t. what? are they comparable with the
% ones in suit.link related to the base?

%% Load and read humanState file

bucket.LOGfilename = fullfile(bucket.pathToIKdata, 'data.log');
fileID = fopen(bucket.LOGfilename);
formatSpec = '%s';
tmp.IKfile = textscan(fileID, formatSpec,'MultipleDelimsAsOne', 1, 'Delimiter', {' '});
fclose(fileID);

% Remove and replace symbols
tmp.match = (')');
tmp.IKfile{1,1} = erase(tmp.IKfile{1,1},tmp.match);
tmp.match = ('(');
tmp.IKfile{1,1} = erase(tmp.IKfile{1,1},tmp.match);

%% Create data struct
IKdata =[];

% Detect the first joint label for the pattern
tmp.match = ('_');
tmp.tmpIKfile{1,1} = erase(tmp.IKfile{1,1},tmp.match);
for fileIdx = 1 : length(tmp.IKfile{1,1})
    if isstrprop(tmp.tmpIKfile{1, 1}{fileIdx,1},'alphanum')
        tmp.firstPatternLabelIdx = fileIdx;
        break;
    end
end
tmp.firstPatternLabel = tmp.IKfile{1,1}{tmp.firstPatternLabelIdx,1};
tmp.patternLabelIndx = find(strcmp(tmp.IKfile{1, 1}, tmp.firstPatternLabel));
IKdata.nrOfFrames   = length(tmp.patternLabelIndx);

% --------TIMESTAMP
% The timestamp appears always before the string firstPatternLabel
IKdata.timestamp = zeros(IKdata.nrOfFrames,1);
for tsIdx = 1 : IKdata.nrOfFrames
    IKdata.timestamp(tsIdx,1) = str2num(tmp.IKfile{1, 1}{tmp.patternLabelIndx(tsIdx)-1,1});
end

% --------FRAME RATE
tmp.timestampNormalized = IKdata.timestamp - IKdata.timestamp(1,1);
IKdata.estimatedFrameRate  = round(mean(1./(diff(tmp.timestampNormalized))));

%% JOINTS
% Consistency check
tmp.setOfLabelsIdx = [ ];
for fileIdx = 1 : length(tmp.IKfile{1,1})
    if isstrprop(tmp.tmpIKfile{1, 1}{fileIdx,1},'alphanum')
        tmp.setOfLabelsIdx = [tmp.setOfLabelsIdx, fileIdx];
    end
end

if diff(tmp.setOfLabelsIdx(1:66)) == 1
    IKdofs= 66;
else
    IKdofs= 48;
end

if IKdofs ~= nrDofs
    error('Dofs of the IK are not consistent with the analysis! Check it! ...')
end

selectedJoints = cell(nrDofs,1);
for nrDofsIdx = 1 : nrDofs
    % selectedJoints
    selectedJoints{nrDofsIdx} = tmp.IKfile{1, 1}{nrDofsIdx+3,1};
    
    IKdata.joints{nrDofsIdx,1}.label    = selectedJoints{nrDofsIdx};
    IKdata.joints{nrDofsIdx,1}.angle    = zeros(1,IKdata.nrOfFrames);
    IKdata.joints{nrDofsIdx,1}.velocity = zeros(1,IKdata.nrOfFrames);
    for frameIdx = 1 : IKdata.nrOfFrames
        % joint angles
        IKdata.joints{nrDofsIdx,1}.angle(frameIdx) = ...
            str2num(tmp.IKfile{1, 1}{tmp.patternLabelIndx(frameIdx)+(nrDofs+nrDofsIdx-1),1});
        % joint veloicity
        IKdata.joints{nrDofsIdx,1}.velocity(frameIdx) = ...
            str2num(tmp.IKfile{1, 1}{tmp.patternLabelIndx(frameIdx)+((2*nrDofs)+nrDofsIdx-1),1});
    end
end

%% BASE
IKdata.base.label = tmp.IKfile{1, 1}{tmp.patternLabelIndx(frameIdx)+(3*nrDofs),1};
IKdata.base.quaternion      = zeros(4,IKdata.nrOfFrames);
IKdata.base.pos             = zeros(3,IKdata.nrOfFrames);
IKdata.base.linearVelocity  = zeros(3,IKdata.nrOfFrames);
IKdata.base.angularVelocity = zeros(3,IKdata.nrOfFrames);
for frameIdx = 1 : IKdata.nrOfFrames
    for vect3Idx = 1 : 3  % for 3D vectors
        % position
        IKdata.base.pos(vect3Idx,frameIdx) = ...
            str2num(tmp.IKfile{1, 1}{tmp.patternLabelIndx(frameIdx)+(3*nrDofs+4) + (vect3Idx),1});
        % linear velocity
        IKdata.base.linearVelocity(vect3Idx,frameIdx) = ...
            str2num(tmp.IKfile{1, 1}{tmp.patternLabelIndx(frameIdx)+(3*nrDofs+7) + (vect3Idx),1});
        % angular velocity
        IKdata.base.angularVelocity(vect3Idx,frameIdx) = ...
            str2num(tmp.IKfile{1, 1}{tmp.patternLabelIndx(frameIdx)+(3*nrDofs+10) + (vect3Idx),1});
    end
    for vect4Idx = 1 : 4 % for quaternions
        IKdata.base.quaternion(vect4Idx,frameIdx) = ...
            str2num(tmp.IKfile{1, 1}{tmp.patternLabelIndx(frameIdx)+(3*nrDofs) + (vect4Idx),1});
    end
end

%% Cleaning up 
clearvars tmp;
