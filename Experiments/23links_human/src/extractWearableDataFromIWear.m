
%% Structure of the data
% Per each timestamp the wearable .log file provides data according to
% the following structure:

% STATUS: 1 value
% -----
% UNKNOWN: 1 value, if data are dumped with yarpdatadumper rx/tx time (discard it!)
% -----
% TIMPESTAMP: 1 value
% -----
% DEVICE NAME: IWearRemapper
% -----
% (struct Accelerometer{})  -->empty
% -----
% (struct EmgSensor) -->empty
% -----
% (struct Force3Dsensor{}) -->empty
% -----
% (struct ForceTorque6DSensor{}):
%        (label_left (label_left  STATUS f_x f_y f_z
%                                        m_x m_y m_z))
%        (label_right(label_right STATUS f_x f_y f_z
%                                        m_x m_y m_z))
% -----
% (struct FreeBodyAccelrationSensor{}):
%        (XsensSuit::fbAcc::sensorName1 (XsensSuit::fbAcc::sensorName1 STATUS fba_x fba_y fba_z))
%            .
%            .
%            .
%        (XsensSuit::fbAcc::sensorName17 (XsensSuit::fbAcc::sensorName17 STATUS fba_x fba_y fba_z))
% -----
% (struct Gyroscope{}) -->empty
% -----
% (struct Magnetometer{}):
%        (XsensSuit::mag::sensorName1 (XsensSuit::mag::sensorName1 STATUS mag_x mag_y mag_z))
%            .
%            .
%            .
%        (XsensSuit::mag::sensorName17 (XsensSuit::mag::sensorName17 STATUS mag_x mag_y mag_z))
% -----
% (struct OrientationSensor{}):
%        (XsensSuit::orient::sensorName1 (XsensSuit::orient::sensorName1 STATUS orient_w orient_x orient_y orient_z))
%            .
%            .
%            .
%        (XsensSuit::orient::sensorName17 (XsensSuit::orient::sensorName17 STATUS orient_w orient_x orient_y orient_z))
% -----
% (struct PoseSensor{}):
%        (XsensSuit::pose::sensorName1 (XsensSuit::pose::sensorName1 STATUS pos_x pos_y pos_z
%                                                                           orient_w orient_x orient_y orient_z))
%            .
%            .
%            .
%        (XsensSuit::pose::sensorName17 (XsensSuit::pose::sensorName17 STATUS pos_x pos_y pos_z
%                                                                             orient_w orient_x orient_y orient_z))
% -----
% (struct PositionSensor{}):
%        (XsensSuit::pos::sensorName1 (XsensSuit::pos::sensorName1 STATUS pos_x pos_y pos_z))
%            .
%            .
%            .
%        (XsensSuit::pos::sensorName17 (XsensSuit::pos::sensorName17 STATUS pos_x pos_y pos_z))
% -----
% (struct skinSensor{})  -->empty
% -----
% (struct TemperatureSensor) -->empty
% -----
% (struct Torque3Dsensor{}) -->empty
% -----
% (struct VirtualLinkKinSensor{}):
%        (XsensSuit::vLink::linkName1 (XsensSuit::vLink::linkName1 STATUS orient_w orient_x orient_y orient_z
%                                                                         pos_x pos_y pos_z
%                                                                         linVel_x linVel_y linVel_z
%                                                                         angVel_x angVel_y angVel_z
%                                                                         linAcc_x linAcc_y pos_z
%                                                                         angAcc_x angAcc_y angAcc_z))
%            .
%            .
%            .
%        (XsensSuit::vLink::linkName23 (XsensSuit::vLink::linkName23 STATUS orient_w orient_x orient_y orient_z
%                                                                           pos_x pos_y pos_z
%                                                                           linVel_x linVel_y linVel_z
%                                                                           angVel_x angVel_y angVel_z
%                                                                           linAcc_x linAcc_y pos_z
%                                                                           angAcc_x angAcc_y angAcc_z))
% -----
% (struct VirtualSphericalJointKinSensor{}):
%        (XsensSuit::vSJoint::jointName1 (XsensSuit::vSJoint::jointName1 STATUS q_x q_y q_z
%                                                                                 dq_x dq_y dq_z
%                                                                                 ddq_x ddq_y ddq_z))
%            .
%            .
%            .
%        (XsensSuit::vSJoint::jointName22 (XsensSuit::vSJoint::jointName22 STATUS q_x q_y q_z
%                                                                                 dq_x dq_y dq_z
%                                                                                 ddq_x ddq_y ddq_z))

%% Load and read wearable file

bucket.LOGfilename = fullfile(bucket.pathToWearableData, 'data.log');
fileID = fopen(bucket.LOGfilename);
formatSpec = '%s';
tmp.file = textscan(fileID, formatSpec,'MultipleDelimsAsOne', 1, 'Delimiter', {' '});
fclose(fileID);

% Remove and replace symbols
tmp.match = ('((');
tmp.file{1,1} = erase(tmp.file{1,1},tmp.match);
tmp.match = ('))');
tmp.file{1,1} = erase(tmp.file{1,1},tmp.match);
tmp.match = ('"');
tmp.file{1,1} = erase(tmp.file{1,1},tmp.match);

tmp.oldStr = '()';
tmp.newStr = 'empty';
tmp.file{1,1} = replace(tmp.file{1,1}, tmp.oldStr, tmp.newStr);

tmp.match = (')');
tmp.file{1,1} = erase(tmp.file{1,1},tmp.match);
tmp.match = ('(');
tmp.file{1,1} = erase(tmp.file{1,1},tmp.match);

for fileIdx = 1 : length(tmp.file{1,1})-1
    if strcmp(tmp.file{1, 1}{fileIdx,1},tmp.file{1, 1}{fileIdx+1,1})
        TF = isstrprop(tmp.file{1, 1}{fileIdx,1},'digit');
        if ~any(TF)
            tmp.file{1, 1}{fileIdx,1} = 'repeatedValue';
        end
    end
end

%% Create data struct
wearData =[];
% --------PROPERTIES
wearData.properties.nrOfLinks    = 23;
wearData.properties.nrOfJoints   = 22;
wearData.properties.nrOfSensors  = 17;

% --------TIMESTAMP
% The timestamp appears always before the string 'IWearRemapper'
tmp.IWearRemapperIndx = find(strcmp(tmp.file{1, 1}, 'IWearRemapper'));
wearData.nrOfFrames   = length(tmp.IWearRemapperIndx);

wearData.timestamp = zeros(wearData.nrOfFrames,1);
for tsIdx = 1 : wearData.nrOfFrames
    wearData.timestamp(tsIdx,1) = str2num(tmp.file{1, 1}{tmp.IWearRemapperIndx(tsIdx)-1,1});
end

% --------FRAME RATE
tmp.timestampNormalized = wearData.timestamp - wearData.timestamp(1,1);
wearData.estimatedFrameRate  = round(mean(1./(diff(tmp.timestampNormalized))));

%% FTSHOES
tmp.leftFtShoeIndx  = find(strcmp(tmp.file{1, 1}, 'FTShoeLeftFTSensors'));
tmp.rightFtShoeIndx = find(strcmp(tmp.file{1, 1}, 'FTShoeRightFTSensors'));

% ----Check if values are missing in the left shoe
flag.missingLeftValues = false; % assumed no value is missing as default
if length(tmp.leftFtShoeIndx) ~= wearData.nrOfFrames
    disp('[Info]: Missing values for the Left shoe!');
    [tmp.missingLeftShoeIdx,flag.missingLeftValues] = checkingMissingValues(tmp.file{1, 1}, ...
        tmp.IWearRemapperIndx, wearData.nrOfFrames, 5, 'FTShoeLeftFTSensors');
end

% ----Check if values are missing in the right shoe
flag.missingRightValues = false; % assumed no value is missing as default
if length(tmp.rightFtShoeIndx) ~= wearData.nrOfFrames
    disp('[Info]: Missing values for the Right shoe!');
    [tmp.missingRightShoeIdx,flag.missingRightValues] = checkingMissingValues(tmp.file{1, 1}, ...
        tmp.IWearRemapperIndx, wearData.nrOfFrames, 14, 'FTShoeRightFTSensors');
end

% ----Fill wearData for the shoes
wearData.ftShoes.Left  = zeros(6,length(tmp.leftFtShoeIndx));
wearData.ftShoes.Right = zeros(6,length(tmp.rightFtShoeIndx));
for l_shoeIdx = 1 : length(tmp.leftFtShoeIndx) % Left
    for vect6Idx = 1 : 6
        wearData.ftShoes.Left(vect6Idx,l_shoeIdx)  = str2num(tmp.file{1, 1}{tmp.leftFtShoeIndx(l_shoeIdx)+(vect6Idx+1),1});
    end
end
for r_shoeIdx = 1 : length(tmp.rightFtShoeIndx) % Right
    for vect6Idx = 1 : 6
        wearData.ftShoes.Right(vect6Idx,r_shoeIdx)  = str2num(tmp.file{1, 1}{tmp.rightFtShoeIndx(r_shoeIdx)+(vect6Idx+1),1});
    end
end

% ----If values missing, fill them with the previous valid one
% Left
if flag.missingLeftValues
    wearData.ftShoes.Left = fillMissingValues(tmp.leftFtShoeIndx,tmp.missingLeftShoeIdx,wearData.ftShoes.Left);
end
% Right
if flag.missingRightValues
    wearData.ftShoes.Right = fillMissingValues(tmp.rightFtShoeIndx,tmp.missingRightShoeIdx,wearData.ftShoes.Right);
end

% ----Final shoes check: wearData.ftShoes.Right and wearData.ftShoes.Left
% must have the same dimension
if length(wearData.ftShoes.Right) ~= length(wearData.ftShoes.Left)
    error('Different length for the shoes! Check it! ...')
end

%% SENSORS
listOfSensorsLabel = {'Head', ...
    'LeftFoot', ...
    'LeftForeArm', ...
    'LeftHand', ...
    'LeftLowerLeg', ...
    'LeftShoulder', ...
    'LeftUpperArm', ...
    'LeftUpperLeg', ...
    'Pelvis', ...
    'RightFoot', ...
    'RightForeArm', ...
    'RightHand', ...
    'RightLowerLeg', ...
    'RightShoulder', ...
    'RightUpperArm', ...
    'RightUpperLeg', ...
    'T8'};
wearData.sensors = cell(wearData.properties.nrOfSensors, 1);

for sensIdx = 1 : wearData.properties.nrOfSensors
    wearData.sensors{sensIdx}.label = listOfSensorsLabel{sensIdx};
    wearData.sensors{sensIdx}.attachedLink = wearData.sensors{sensIdx}.label;
    
    tmp.fbAccIdx  = find(strcmp(tmp.file{1, 1}, strcat('XsensSuit::fbAcc::',listOfSensorsLabel{sensIdx})));
    wearData.sensors{sensIdx}.meas.sensorFreeAcceleration = zeros(3,wearData.nrOfFrames);
    
    tmp.magIdx  = find(strcmp(tmp.file{1, 1}, strcat('XsensSuit::mag::',listOfSensorsLabel{sensIdx})));
    wearData.sensors{sensIdx}.meas.sensorMagneticField = zeros(3,wearData.nrOfFrames);
    
    tmp.orientIdx  = find(strcmp(tmp.file{1, 1}, strcat('XsensSuit::orient::',listOfSensorsLabel{sensIdx})));
    wearData.sensors{sensIdx}.meas.sensorOrientation = zeros(4,wearData.nrOfFrames);
    
    tmp.poseIdx  = find(strcmp(tmp.file{1, 1}, strcat('XsensSuit::pose::',listOfSensorsLabel{sensIdx})));
    wearData.sensors{sensIdx}.meas.pose.sensorPositionWRTglobal = zeros(3,wearData.nrOfFrames);
    wearData.sensors{sensIdx}.meas.pose.sensorOrientation       = zeros(4,wearData.nrOfFrames);
    
    tmp.posIdx  = find(strcmp(tmp.file{1, 1}, strcat('XsensSuit::pos::',listOfSensorsLabel{sensIdx})));
    wearData.sensors{sensIdx}.meas.sensorPositionWRTglobal = zeros(3,wearData.nrOfFrames);
    
    for frameIdx = 1 : wearData.nrOfFrames
        for vect3Idx = 1 : 3 % for 3D vectors
            % sensorFreeAcceleration
            wearData.sensors{sensIdx}.meas.sensorFreeAcceleration(vect3Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.fbAccIdx(frameIdx)+(vect3Idx+1),1});
            % sensorMagneticField
            wearData.sensors{sensIdx}.meas.sensorMagneticField(vect3Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.magIdx(frameIdx)+(vect3Idx+1),1});
            % pose.sensorPositionWRTglobal
            wearData.sensors{sensIdx}.meas.pose.sensorPositionWRTglobal(vect3Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.poseIdx(frameIdx)+(vect3Idx+5),1});
            % sensorPositionWRTglobal
            wearData.sensors{sensIdx}.meas.sensorPositionWRTglobal(vect3Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.posIdx(frameIdx)+(vect3Idx+1),1});
            
            % NOTE: pose.sensorPositionWRTglobal and sensorPositionWRTglobal are the same quantities.
        end
        for vect4Idx = 1 : 4 % for quaternions
            % sensorOrientation
            wearData.sensors{sensIdx}.meas.sensorOrientation(vect4Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.orientIdx(frameIdx)+(vect4Idx+1),1});
            % pose.sensorOrientation
            wearData.sensors{sensIdx}.meas.pose.sensorOrientation(vect4Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.poseIdx(frameIdx)+(vect4Idx+1),1});
            
            % NOTE: sensorOrientation and pose.sensorOrientation are the same quantities.
        end
    end
end

%% LINKS
listOfLinksLabel = {'Head', ...
    'L3', ...
    'L5', ...
    'LeftFoot', ...
    'LeftForeArm', ...
    'LeftHand', ...
    'LeftLowerLeg', ...
    'LeftShoulder', ...
    'LeftToe', ...
    'LeftUpperArm', ...
    'LeftUpperLeg', ...
    'Neck', ...
    'Pelvis', ...
    'RightFoot', ...
    'RightForeArm', ...
    'RightHand', ...
    'RightLowerLeg', ...
    'RightShoulder', ...
    'RightToe', ...
    'RightUpperArm', ...
    'RightUpperLeg', ...
    'T12', ...
    'T8'};
wearData.links = cell(wearData.properties.nrOfLinks, 1);

for linkIdx = 1 : wearData.properties.nrOfLinks
    wearData.links{linkIdx}.label = listOfLinksLabel{linkIdx};
    
    tmp.vLinkIdx  = find(strcmp(tmp.file{1, 1}, strcat('XsensSuit::vLink::',listOfLinksLabel{linkIdx})));
    wearData.links{linkIdx}.meas.orientation         = zeros(4,wearData.nrOfFrames);
    wearData.links{linkIdx}.meas.position            = zeros(3,wearData.nrOfFrames);
    wearData.links{linkIdx}.meas.velocity            = zeros(3,wearData.nrOfFrames);
    wearData.links{linkIdx}.meas.angularVelocity     = zeros(3,wearData.nrOfFrames);
    wearData.links{linkIdx}.meas.acceleration        = zeros(3,wearData.nrOfFrames);
    wearData.links{linkIdx}.meas.angularAcceleration = zeros(3,wearData.nrOfFrames);
    % no points from XsensSuit are outputted.
    
    for frameIdx = 1 : wearData.nrOfFrames
        for vect4Idx = 1 : 4 % for quaternions
            % orientation
            wearData.links{linkIdx}.meas.orientation(vect4Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.vLinkIdx(frameIdx)+(vect4Idx+1),1});
        end
        for vect3Idx = 1 : 3 % for 3D vectors
            % position
            wearData.links{linkIdx}.meas.position(vect3Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.vLinkIdx(frameIdx)+(vect3Idx+5),1});
            % velocity
            wearData.links{linkIdx}.meas.velocity(vect3Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.vLinkIdx(frameIdx)+(vect3Idx+8),1});
            % angularVelocity
            wearData.links{linkIdx}.meas.angularVelocity(vect3Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.vLinkIdx(frameIdx)+(vect3Idx+11),1});
            % acceleration
            wearData.links{linkIdx}.meas.acceleration(vect3Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.vLinkIdx(frameIdx)+(vect3Idx+14),1});
            % angularAcceleration
            wearData.links{linkIdx}.meas.angularAcceleration(vect3Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.vLinkIdx(frameIdx)+(vect3Idx+17),1});
        end
    end
end

%% JOINTS
listOfJointsLabel = {'jC1Head', ...
    'jL1T12', ...
    'jL4L3', ...
    'jL5S1', ...
    'jLeftAnkle', ...
    'jLeftBallFoot', ...
    'jLeftElbow', ...
    'jLeftHip', ...
    'jLeftKnee', ...
    'jLeftShoulder', ...
    'jLeftT4Shoulder', ...
    'jLeftWrist', ...
    'jRightAnkle', ...
    'jRightBallFoot', ...
    'jRightElbow', ...
    'jRightHip', ...
    'jRightKnee', ...
    'jRightShoulder', ...
    'jRightT4Shoulder', ...
    'jRightWrist', ...
    'jT1C7', ...
    'jT9T8'};
wearData.joints = cell(wearData.properties.nrOfJoints, 1);

for jointIdx = 1 : wearData.properties.nrOfJoints
    wearData.joints{jointIdx}.label = listOfJointsLabel{jointIdx};
    
    tmp.vJointIdx  = find(strcmp(tmp.file{1, 1}, strcat('XsensSuit::vSJoint::',listOfJointsLabel{jointIdx})));
    wearData.joints{jointIdx}.meas.angle        = zeros(3,wearData.nrOfFrames);
    %     wearData.joints{jointIdx}.meas.velocity     = zeros(3,wearData.nrOfFrames);
    %     wearData.joints{jointIdx}.meas.acceleration = zeros(3,wearData.nrOfFrames);
    
    for frameIdx = 1 : wearData.nrOfFrames
        for vect3Idx = 1 : 3 % for 3D vectors
            % angle
            wearData.joints{jointIdx}.meas.angle(vect3Idx,frameIdx) = ...
                str2num(tmp.file{1, 1}{tmp.vJointIdx(frameIdx)+(vect3Idx+1),1});
        end
    end
end

%% Cleaning up
clearvars tmp;
