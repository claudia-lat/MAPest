
%% Load and read parsed file
bucket.pathToSuitData   = fullfile(bucket.pathToRawData,'parsedFromMvnx');
% -----
% from XML file:
% generic info, points, identity/tpose/tpose-isb
bucket.XMLfilename = fullfile(bucket.pathToSuitData, sprintf('S%02d_%02d.xml',subjectID,taskID));
mvnxData = xml_read(bucket.XMLfilename);
% -----
% from LOG file:
% segment and sensor list
bucket.LOGfilename = fullfile(bucket.pathToSuitData, sprintf('S%02d_%02d.log',subjectID,taskID));
fileID = fopen(bucket.LOGfilename);
formatSpec = '%s';
tmp.dummyParsedMatrix = textscan(fileID, formatSpec,'MultipleDelimsAsOne', 1, 'Delimiter', {','});
fclose(fileID);
% -----
% from CSV file:
% index, msTime, xSensTime, each link (position, acceleration, orientation, angular
% velocity, angular acceleration), each sensor (orientation, free acceleration).
% NOTE: This file is used for the MODEL CREATION and it is loaded and
% processed once (i.e., one trial) per each subject!
bucket.CSVfilename = fullfile(bucket.pathToSuitData, sprintf('S%02d_%02d.csv',subjectID,taskID));
mvnxDataFromCSV.data         = table2array(readtable(bucket.CSVfilename,'Delimiter',',')); %array
mvnxDataFromCSV.orderedLabel = (getListFromCSV(bucket.CSVfilename,1,1,size(mvnxDataFromCSV.data,2)))'; %list of char

%% Create data struct
suit =[];
% --------PROPERTIES
suit.properties.experimentLabel = mvnxData.ATTRIBUTE.label;
suit.properties.recordingDate   = mvnxData.ATTRIBUTE.recDate;
suit.properties.frameRate       = mvnxData.ATTRIBUTE.frameRate;
suit.properties.nrOfLinks       = mvnxData.frames.ATTRIBUTE.segmentCount;
suit.properties.nrOfJoints      = mvnxData.frames.ATTRIBUTE.jointCount;
suit.properties.nrOfSensors     = mvnxData.frames.ATTRIBUTE.sensorCount;
nrOfCalibFrames                 = 3;
nrOfFrames                      = size(mvnxDataFromCSV.data,1); % without the 3 calib frames
suit.properties.lenData         = nrOfFrames;
% --------CALIBRATION
suit.calibration = struct;
% --------TIME
suit.time.ms    = zeros(1,suit.properties.lenData); % time in ms
suit.time.xSens = zeros(1,suit.properties.lenData); % time indexed by Xsens
% % --------COM %not used in JSI analysis
% suit.COM  = zeros(3,suit.properties.lenData);
% --------LINKS
suit.links = cell(suit.properties.nrOfLinks, 1);
for i = 1 : suit.properties.nrOfLinks
    suit.links{i}.id = mvnxData.segments.segment(i).ATTRIBUTE.id;
    suit.links{i}.label = mvnxData.segments.segment(i).ATTRIBUTE.label;
    suit.links{i}.meas = struct;
    suit.links{i}.meas.orientation         = zeros(4, suit.properties.lenData);
    suit.links{i}.meas.position           = zeros(3, suit.properties.lenData);
    suit.links{i}.meas.velocity           = zeros(3, suit.properties.lenData);
    suit.links{i}.meas.acceleration        = zeros(3, suit.properties.lenData);
    suit.links{i}.meas.angularVelocity     = zeros(3, suit.properties.lenData);
    suit.links{i}.meas.angularAcceleration = zeros(3, suit.properties.lenData);
    suit.links{i}.points                   = struct;
    suit.links{i}.points.nrOfPoints        = size(mvnxData.segments.segment(i).points.point,1);
    suit.links{i}.points.pointsValue       = zeros(3,suit.links{i}.points.nrOfPoints);
    for k = 1 : suit.links{i}.points.nrOfPoints
        suit.links{i}.points.label(1,k) = cellstr(mvnxData.segments.segment(i).points.point(k).ATTRIBUTE.label);
        suit.links{i}.points.pointsValue(:,k) = mvnxData.segments.segment(i).points.point(k).pos_b;
    end
end
% --------JOINTS % not used in JSI analysis
% suit.joints = cell(suit.properties.nrOfJoints,1);
% for i = 1 : suit.properties.nrOfJoints
%     suit.joints{i}.label              = mvnxData.joints.joint(i).ATTRIBUTE.label;
%     suit.joints{i}.meas               = struct;
%     suit.joints{i}.meas.jointAngle    = zeros(3, suit.properties.lenData);
%     suit.joints{i}.meas.jointAngleXZY = zeros(3, suit.properties.lenData);
% end
% --------SENSORS
suit.sensors = cell(suit.properties.nrOfSensors,1);
for i = 1 : suit.properties.nrOfSensors
    suit.sensors{i}.label                      = mvnxData.sensors.sensor(i).ATTRIBUTE.label;
    suit.sensors{i}.attachedLink               = suit.sensors{i}.label; % assumption: the label of the sensor is the same one of the link on which the sensor is attached
    suit.sensors{i}.meas.sensorOrientation     = zeros(4, suit.properties.lenData);
    suit.sensors{i}.meas.sensorFreeAcceleration = zeros(3, suit.properties.lenData);
    %     suit.sensors{i}.meas.sensorMagneticField    = zeros(3, suit.properties.lenData); %not used in JSI analysis, not in the.csv
end

%% Fill the struct with recording data
% --------SEGMENT AND SENSOR LISTS
% this info is not into the suit struct but is useful for further analysis.
for  i = 1 : size(tmp.dummyParsedMatrix{1, 1},1)
    if (strcmp(tmp.dummyParsedMatrix{1, 1}{i, 1}, 'SegmentsList'))
        tmp.segmentListIdx = i;
    end
    if (strcmp(tmp.dummyParsedMatrix{1, 1}{i, 1}, 'SensorsList'))
        tmp.sensorListIdx = i;
        break;
    end
end

segmentList = cell(suit.properties.nrOfLinks,1);
for suitLinkIdx = 1 : suit.properties.nrOfLinks
    segmentList{suitLinkIdx,1} = tmp.dummyParsedMatrix{1, 1}{suitLinkIdx+1, 1};
end
sensorList = cell(suit.properties.nrOfSensors,1);
for suitSensorIdx = 1 : suit.properties.nrOfSensors
    sensorList{suitSensorIdx,1} = tmp.dummyParsedMatrix{1, 1}{suitSensorIdx+1, 1};
end
% --------CALIBRATION
% calibration data obtained from XML file
a = 4; %dimension of quaternions
b = 3; %dimension of vectors
for frameIdx = 1 : nrOfCalibFrames
    currentFrame = mvnxData.frames.frame(frameIdx);
    % identity FIELD
    if (strcmp(mvnxData.frames.frame(frameIdx).ATTRIBUTE.type, 'identity'))
        suit.calibration.identity             = struct;
        %       suit.calibration.identity.index       = -3;
        suit.calibration.identity.orientation = zeros(4, suit.properties.nrOfLinks);
        suit.calibration.identity.position    = zeros(3, suit.properties.nrOfLinks);
        for i = 1 : suit.properties.nrOfLinks
            suit.calibration.identity.orientation(:,i) = currentFrame.orientation(1, a*(i-1)+1 : a*i);
            suit.calibration.identity.position(:,i)    = currentFrame.position(1, b*(i-1)+1 : b*i);
        end
        continue;
    end
    % Tpose FIELD
    if (strcmp(mvnxData.frames.frame(frameIdx).ATTRIBUTE.type, 'tpose'))
        suit.calibration.tpose             = struct;
        %       suit.calibration.tpose.index       = -2;
        suit.calibration.tpose.orientation = zeros(4, suit.properties.nrOfLinks);
        suit.calibration.tpose.position    = zeros(3, suit.properties.nrOfLinks);
        for i = 1 : suit.properties.nrOfLinks
            suit.calibration.tpose.orientation(:,i) = currentFrame.orientation(1, a*(i-1)+1 : a*i);
            suit.calibration.tpose.position(:,i)    = currentFrame.position(1, b*(i-1)+1 : b*i);
        end
        continue;
    end
    % Tpose-isb FIELD
    if (strcmp(mvnxData.frames.frame(frameIdx).ATTRIBUTE.type, 'tpose-isb'))
        suit.calibration.tpose_isb             = struct;
        %       suit.calibration.tpose_isb.index       = -1;
        suit.calibration.tpose_isb.orientation = zeros(4, suit.properties.nrOfLinks);
        suit.calibration.tpose_isb.position    = zeros(3, suit.properties.nrOfLinks);
        for i = 1 : suit.properties.nrOfLinks
            suit.calibration.tpose_isb.orientation(:,i) = currentFrame.orientation(1, a*(i-1)+1 : a*i);
            suit.calibration.tpose_isb.position(:,i)    = currentFrame.position(1, b*(i-1)+1 : b*i);
        end
        continue;
    end
end
% --------TIME
if ~isempty(suit.time)
    lenCSVlabel = size(mvnxDataFromCSV.orderedLabel,1);
    for frameIdx = 1 : nrOfFrames
        for labelIdx = 1 : lenCSVlabel
            if (strcmp(mvnxDataFromCSV.orderedLabel{labelIdx,1}, 'msTime'))
                suit.time.ms = mvnxDataFromCSV.data(:,labelIdx)';
            end
            if (strcmp(mvnxDataFromCSV.orderedLabel{labelIdx,1}, 'xSensTime'))
                suit.time.xSens = mvnxDataFromCSV.data(:,labelIdx)';
            end
        end
    end
end
%--------INTERMEDIATE STRUCT FROM CSV
%--LINKS
for i = 1: size(mvnxDataFromCSV.orderedLabel,1)
    if (contains(mvnxDataFromCSV.orderedLabel{i, 1}, 'position:'))
        tmp.lastIdxPos = i; %last index
    end
    if (contains(mvnxDataFromCSV.orderedLabel{i, 1}, 'velocity:'))
        tmp.lastIdxVel = i; %last index
    end
    if (contains(mvnxDataFromCSV.orderedLabel{i, 1}, 'acceleration:'))
        tmp.lastIdxAcc = i; %last index
    end
    if (contains(mvnxDataFromCSV.orderedLabel{i, 1}, 'orientation:'))
        tmp.lastIdxOr = i; %last index
    end
    if (contains(mvnxDataFromCSV.orderedLabel{i, 1}, 'angularVelocity:'))
        tmp.lastIdxAngVel = i; %last index
    end
    if (contains(mvnxDataFromCSV.orderedLabel{i, 1}, 'angularAcceleration:'))
        tmp.lastIdxAngAcc = i; %last index
    end
end
% (23x3 variables)
tmp.link.position.orderedLabel = cell(3*(suit.properties.nrOfLinks),1);
tmp.link.position.data =zeros(nrOfFrames,3*(suit.properties.nrOfLinks));

tmp.link.velocity.orderedLabel = cell(3*(suit.properties.nrOfLinks),1);
tmp.link.velocity.data = zeros(nrOfFrames,3*(suit.properties.nrOfLinks));

tmp.link.acceleration.orderedLabel = cell(3*(suit.properties.nrOfLinks),1);
tmp.link.acceleration.data = zeros(nrOfFrames,3*(suit.properties.nrOfLinks));

tmp.link.angularVelocity.orderedLabel = cell(3*(suit.properties.nrOfLinks),1);
tmp.link.angularVelocity.data = zeros(nrOfFrames,3*(suit.properties.nrOfLinks));

tmp.link.angularAcceleration.orderedLabel = cell(3*(suit.properties.nrOfLinks),1);
tmp.link.angularAcceleration.data = zeros(nrOfFrames,3*(suit.properties.nrOfLinks));

for i = 1 : 3*(suit.properties.nrOfLinks)
    tmp.link.position.orderedLabel{i,1} = mvnxDataFromCSV.orderedLabel{i+(tmp.lastIdxPos - 3*(suit.properties.nrOfLinks)), 1};
    tmp.link.position.data(:,i) = mvnxDataFromCSV.data(:,i+(tmp.lastIdxPos - 3*(suit.properties.nrOfLinks)));
    
    tmp.link.velocity.orderedLabel{i,1} = mvnxDataFromCSV.orderedLabel{i+(tmp.lastIdxVel - 3*(suit.properties.nrOfLinks)), 1};
    tmp.link.velocity.data(:,i) = mvnxDataFromCSV.data(:,i+(tmp.lastIdxVel - 3*(suit.properties.nrOfLinks)));
    
    tmp.link.acceleration.orderedLabel{i,1} = mvnxDataFromCSV.orderedLabel{i+(tmp.lastIdxAcc - 3*(suit.properties.nrOfLinks)), 1};
    tmp.link.acceleration.data(:,i) = mvnxDataFromCSV.data(:,i+(tmp.lastIdxAcc - 3*(suit.properties.nrOfLinks)));
    
    tmp.link.angularVelocity.orderedLabel{i,1} = mvnxDataFromCSV.orderedLabel{i+(tmp.lastIdxAngVel - 3*(suit.properties.nrOfLinks)), 1};
    tmp.link.angularVelocity.data(:,i) = mvnxDataFromCSV.data(:,i+(tmp.lastIdxAngVel - 3*(suit.properties.nrOfLinks)));
    
    tmp.link.angularAcceleration.orderedLabel{i,1} = mvnxDataFromCSV.orderedLabel{i+(tmp.lastIdxAngAcc - 3*(suit.properties.nrOfLinks)), 1};
    tmp.link.angularAcceleration.data(:,i) = mvnxDataFromCSV.data(:,i+(tmp.lastIdxAngAcc - 3*(suit.properties.nrOfLinks)));
end
% (23x4 variables)
tmp.link.orientation.orderedLabel = cell(4*(suit.properties.nrOfLinks),1);
tmp.link.orientation.data = zeros(nrOfFrames,4*(suit.properties.nrOfLinks));

for i = 1 : 4*(suit.properties.nrOfLinks)
    tmp.link.orientation.orderedLabel{i,1} = mvnxDataFromCSV.orderedLabel{i+(tmp.lastIdxOr - 4*(suit.properties.nrOfLinks)), 1};
    tmp.link.orientation.data(:,i) = mvnxDataFromCSV.data(:,i+(tmp.lastIdxOr - 4*(suit.properties.nrOfLinks)));
end
%--SENSORS
for i = 1: size(mvnxDataFromCSV.orderedLabel,1)
    if (contains(mvnxDataFromCSV.orderedLabel{i, 1}, 'sensorOrientation:'))
        tmp.lastIdxSensOr = i; %last index
    end
    if (contains(mvnxDataFromCSV.orderedLabel{i, 1}, 'sensorFreeAcceleration:'))
        tmp.lastIdxSensFreeAcc = i; %last index
    end
end
% (17x4 variables)
tmp.sensor.orientation.orderedLabel = cell(4*(suit.properties.nrOfSensors),1);
tmp.sensor.orientation.data = zeros(nrOfFrames,4*(suit.properties.nrOfSensors));
for i = 1 : 4*(suit.properties.nrOfSensors)
    tmp.sensor.orientation.orderedLabel{i,1} = mvnxDataFromCSV.orderedLabel{i+(tmp.lastIdxSensOr - 4*(suit.properties.nrOfSensors)), 1};
    tmp.sensor.orientation.data(:,i) = mvnxDataFromCSV.data(:,i+(tmp.lastIdxSensOr - 4*(suit.properties.nrOfSensors)));
end
% (17x3 variables)
tmp.sensor.freeAcceleration.orderedLabel = cell(3*(suit.properties.nrOfSensors),1);
tmp.sensor.freeAcceleration.data = zeros(nrOfFrames,3*(suit.properties.nrOfSensors));
for i = 1 : 3*(suit.properties.nrOfSensors)
    tmp.sensor.freeAcceleration.orderedLabel{i,1} = mvnxDataFromCSV.orderedLabel{i+(tmp.lastIdxSensFreeAcc - 3*(suit.properties.nrOfSensors)), 1};
    tmp.sensor.freeAcceleration.data(:,i) = mvnxDataFromCSV.data(:,i+(tmp.lastIdxSensFreeAcc - 3*(suit.properties.nrOfSensors)));
end
%--------FROM TMP TO SUIT
%--LINKS
for suitLinkIdx = 1 : size(suit.links,1)
    % for link pos/vel/acc/angVel/angAcc
    for j = 1:3*(suit.properties.nrOfLinks)
        if (contains(tmp.link.position.orderedLabel{j, 1}, suit.links{suitLinkIdx, 1}.label))
            tmpIndex = j;
            break;
        end
    end
%     for j = 1:3*(suit.properties.nrOfLinks) % for link acc/angVel/angAcc
%         if (contains(tmp.link.acceleration.orderedLabel{j, 1}, suit.links{suitLinkIdx, 1}.label))
%             tmpIndex = j;
%             break;
%         end
%     end
    suit.links{suitLinkIdx}.meas.position            = tmp.link.position.data(:,tmpIndex:tmpIndex+2)';
    suit.links{suitLinkIdx}.meas.velocity            = tmp.link.velocity.data(:,tmpIndex:tmpIndex+2)';
    suit.links{suitLinkIdx}.meas.acceleration        = tmp.link.acceleration.data(:,tmpIndex:tmpIndex+2)';
    suit.links{suitLinkIdx}.meas.angularVelocity     = tmp.link.angularVelocity.data(:,tmpIndex:tmpIndex+2)';
    suit.links{suitLinkIdx}.meas.angularAcceleration = tmp.link.angularAcceleration.data(:,tmpIndex:tmpIndex+2)';
    %  only for orientation
    for j = 1:4*(suit.properties.nrOfLinks)
        if (contains(tmp.link.orientation.orderedLabel{j, 1}, suit.links{suitLinkIdx, 1}.label))
            tmpIndex = j;
            break;
        end
    end
    suit.links{suitLinkIdx}.meas.orientation = tmp.link.orientation.data(:,tmpIndex:tmpIndex+3)';
    
    % !!!!!!!!!!!!!!!! PART TO BE VERIFIED !!!!!!!!!!!!!!!!!!!
    %     currentOrientation = tmp.link.orientation.data(:,tmpIndex:tmpIndex+3)';
    %     %----------------------------------------------------------------------
    %     % IMPORTANT NOTE:
    %     % ---------------
    %     % In general, mvnxData are expressed in Tpose (Fig.60 of manual) with
    %     % the exception of mvnxData.subject.frames.frame.orientation that is
    %     % expressed in a frame (defined 'anatomical') wrt to G.  Please note
    %     % that this anatomical pose A is neither T pose or N pose.
    %     % From data, by using the quaternion as rotation matrix form,
    %     % we have: G_R_A;  we would like to have G_R_T., i.e:
    %     %                    G_R_T =  G_R_A x A_R_T.
    %     %----------------------------------------------------------------------
    % %     if isempty(suit.links{suitLinkIdx}.meas.orientation)
    %         for tmpIdx = 1 : nrOfFrames
    %             %temporary variables
    %             quaternion = iDynTree.Vector4();
    %             rotation   = iDynTree.Rotation();
    %             % get G_R_A matrix from quaternion data
    %             quaternion.fromMatlab(currentOrientation(:,tmpIdx));
    %             rotation.fromQuaternion(quaternion);
    %             G_R_A(:,:,suitLinkIdx) = rotation.toMatlab;
    %             % compute T_R_A using Npose field
    %             quaternion.fromMatlab(suit.calibration.identity.orientation(:,suitLinkIdx));
    %             rotation.fromQuaternion(quaternion);
    %             T_R_A(:,:,suitLinkIdx) = rotation.toMatlab;
    %             % compute A_R_T
    %             A_R_T(:,:,suitLinkIdx) = T_R_A(:,:,suitLinkIdx)';
    %             % compute G_R_T
    %             G_Rot_T(:,:,suitLinkIdx) = G_R_A(:,:,suitLinkIdx) * A_R_T(:,:,suitLinkIdx);
    %             % re-transform G_R_T in quaternion
    %             rotation.fromMatlab(G_Rot_T(:,:,suitLinkIdx));
    %             G_q_T = rotation.asQuaternion();
    %         %
    %         %         % ====test RPY
    %         %         if j == 4998  %Tpose position
    %         %         G_RPY_T(i,:) = rotation.asRPY.toMatlab() / pi * 180;
    %         %         else
    %         %             break
    %         %         end
    %         %         % ===========
    %
    %         %         suit.links{i}.meas.orientation(:,j)         = G_q_T.toMatlab();
    %             suit.links{suitLinkIdx}.meas.orientation(:,tmpIdx) = G_q_T.toMatlab();
    %         end
    % %     end
end
%--SENSORS
for suitSensorIdx = 1 : size(suit.sensors,1)
    % for sensor orientation
    for j = 1:4*(suit.properties.nrOfSensors)
        if (contains(tmp.sensor.orientation.orderedLabel{j, 1}, suit.sensors{suitSensorIdx, 1}.label))
            tmpIndex = j;
            break;
        end
    end
    suit.sensors{suitSensorIdx}.meas.sensorOrientation = tmp.sensor.orientation.data(:,tmpIndex:tmpIndex+3)';
    % for sensor freeAcc
    for j = 1:3*(suit.properties.nrOfSensors)
        if (contains(tmp.sensor.freeAcceleration.orderedLabel{j, 1}, suit.sensors{suitSensorIdx, 1}.label))
            tmpIndex = j;
            break;
        end
    end
    suit.sensors{suitSensorIdx}.meas.sensorFreeAcceleration = tmp.sensor.freeAcceleration.data(:,tmpIndex:tmpIndex+2)';
end

%% Cleaning-up workspace
clearvars a b suitLinkIdx suitSensorIdx tmpIndex i j k formatSpec frameIdx;
