
%% Load and read parsed file
bucket.pathToSuitData   = fullfile(bucket.pathToRawData,'/parsedFromMvnx');
bucket.XMLfilename = sprintf(fullfile(bucket.pathToSuitData,'S%02d_%02d.xml'),subjectID,taskID);
mvnxData = xml_read(bucket.XMLfilename);

bucket.CSVfilename = sprintf(fullfile(bucket.pathToSuitData,'S%02d_%02d.csv'),subjectID,taskID);
bucket.fileToBeSaved = sprintf(fullfile('S%02d_%02d.mat'),subjectID,taskID);
if ~exist(fullfile(bucket.pathToProcessedData,'/',bucket.fileToBeSaved))
    mvnxDataFromCSV.orderedLabel = (importCSVfile(bucket.CSVfilename,1,1))'; %list of strings
    mvnxDataFromCSV.data         = table2array(readtable(bucket.CSVfilename,'Delimiter',',')); %array
    save(fullfile(bucket.pathToProcessedData,'/',bucket.fileToBeSaved),'mvnxDataFromCSV');
else
    load(fullfile(bucket.pathToProcessedData,'/',bucket.fileToBeSaved),'mvnxDataFromCSV');
end

bucket.runtimeFilename = sprintf(fullfile(bucket.pathToSuitData,'S%02d_%02d_runtime.csv'),subjectID,taskID);
bucket.fileToBeSaved = sprintf(fullfile('S%02d_%02d_runtime.mat'),subjectID,taskID);
if ~exist(fullfile(bucket.pathToProcessedData,'/',bucket.fileToBeSaved))
    mvnxDataFromCSV_runtime.orderedLabel = (importCSVfile(bucket.runtimeFilename,1,1))'; %list of strings
    mvnxDataFromCSV_runtime.data         = table2array(readtable(bucket.runtimeFilename,'Delimiter',',')); %array
    save(fullfile(bucket.pathToProcessedData,'/',bucket.fileToBeSaved),'mvnxDataFromCSV_runtime');
else
    load(fullfile(bucket.pathToProcessedData,'/',bucket.fileToBeSaved),'mvnxDataFromCSV_runtime');
end

%% Create data struct
suit =[];
% --------PROPERTIES
suit.properties.experimentLabel = mvnxData.ATTRIBUTE.label;
suit.properties.recordingDate   = mvnxData.ATTRIBUTE.recDate;
suit.properties.frameRate       = mvnxData.ATTRIBUTE.frameRate;
suit.properties.nrOfLinks       = mvnxData.frames.ATTRIBUTE.segmentCount;
suit.properties.nrOfJoints      = mvnxData.frames.ATTRIBUTE.jointCount;
suit.properties.nrOfSensors     = mvnxData.frames.ATTRIBUTE.sensorCount;
nrOfFrames                      = size(mvnxData.frames.frame,1);
suit.properties.lenData         = nrOfFrames;
% --------CALIBRATION
suit.calibration = struct;
counterCalibr = 0;
for j = 1 : suit.properties.lenData
        if (strcmp(mvnxData.frames.frame(j).ATTRIBUTE.type , 'identity'))
            counterCalibr = counterCalibr + 1;
        end
        if (strcmp(mvnxData.frames.frame(j).ATTRIBUTE.type , 'tpose'))
            counterCalibr = counterCalibr + 1;
        end
        if (strcmp(mvnxData.frames.frame(j).ATTRIBUTE.type , 'tpose-isb'))
            counterCalibr = counterCalibr + 1;
        end
end
suit.properties.lenData = suit.properties.lenData - counterCalibr;
% --------TIME
suit.time = zeros(1,suit.properties.lenData);
% --------COM
suit.COM  = zeros(3,suit.properties.lenData);
% --------LINKS
suit.links = cell(suit.properties.nrOfLinks, 1);
for i = 1 : suit.properties.nrOfLinks
    suit.links{i}.id = mvnxData.segments.segment(i).ATTRIBUTE.id;
    suit.links{i}.label = mvnxData.segments.segment(i).ATTRIBUTE.label;
    suit.links{i}.meas = struct;
    suit.links{i}.meas.orientation         = zeros(4, suit.properties.lenData);
    suit.links{i}.meas.position            = zeros(3, suit.properties.lenData);
    suit.links{i}.meas.velocity            = zeros(3, suit.properties.lenData);
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
% --------JOINTS
suit.joints = cell(suit.properties.nrOfJoints,1);
for i = 1 : suit.properties.nrOfJoints
    suit.joints{i}.label              = mvnxData.joints.joint(i).ATTRIBUTE.label;
    suit.joints{i}.meas               = struct;
    suit.joints{i}.meas.jointAngle    = zeros(3, suit.properties.lenData);
    suit.joints{i}.meas.jointAngleXZY = zeros(3, suit.properties.lenData);
end
% --------SENSORS
suit.sensors = cell(suit.properties.nrOfSensors,1);
for i = 1 : suit.properties.nrOfSensors
    suit.sensors{i}.label                      = mvnxData.sensors.sensor(i).ATTRIBUTE.label;
    suit.sensors{i}.attachedLink               = suit.sensors{i}.label; % assumption: the label of the sensor is the same one of the link on which the sensor is attached  
    suit.sensors{i}.meas.sensorOrientation     = zeros(4, suit.properties.lenData);
        suit.sensors{i}.meas.sensorFreeAcceleration = zeros(3, suit.properties.lenData);
        suit.sensors{i}.meas.sensorMagneticField    = zeros(3, suit.properties.lenData);
end

%% Fill the struct with recording data
a = 4; %dimension of quaternions
b = 3; %dimension of vectors
j = 1; %initialize the counter of the frame excluding the calibration frames

for frameIdx = 1 : nrOfFrames
    currentFrame = mvnxData.frames.frame(frameIdx);
    % identity FIELD
    if (strcmp(mvnxData.frames.frame(frameIdx).ATTRIBUTE.type, 'identity'))
        suit.calibration.identity             = struct;
%             suit.calibration.identity.index       = -3;
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
%             suit.calibration.tpose.index       = -2;
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
%             suit.calibration.tpose_isb.index       = -1;
        suit.calibration.tpose_isb.orientation = zeros(4, suit.properties.nrOfLinks);
        suit.calibration.tpose_isb.position    = zeros(3, suit.properties.nrOfLinks);
        for i = 1 : suit.properties.nrOfLinks
            suit.calibration.tpose_isb.orientation(:,i) = currentFrame.orientation(1, a*(i-1)+1 : a*i);
            suit.calibration.tpose_isb.position(:,i)    = currentFrame.position(1, b*(i-1)+1 : b*i);
        end
        continue;
    end
    %----------------------------------------------------------------------
    % IMPORTANT NOTE:
    % ---------------
    % In general, mvnxData are expressed in Tpose (Fig.60 of manual) with
    % the exception of mvnxData.subject.frames.frame.orientation that is
    % expressed in a frame (defined 'anatomical') wrt to G.  Please note
    % that this anatomical pose A is neither T pose or N pose.
    % From data, by using the quaternion as rotation matrix form,
    % we have: G_R_A;  we would like to have G_R_T., i.e:
    %                    G_R_T =  G_R_A x A_R_T.
    %----------------------------------------------------------------------
    % --------TIME
    suit.time(1,j) = currentFrame.ATTRIBUTE.ms;
    % --------COM
    suit.COM(:,j) = currentFrame.centerOfMass';
    % --------CONTACTS
%         suit.contacts = cell(nrOfFrames,1);
%         for i = 4 : nrOfFrames-3
%             suit.contacts{i}(1)     = mvnxData.frames.frame(i).contacts.contact(1).ATTRIBUTE;
%             suit.contacts{i}(2)     = mvnxData.frames.frame(i).contacts.contact(2).ATTRIBUTE;
%         end
    % --------LINKS
    %temporary variables
    quaternion = iDynTree.Vector4();
    rotation   = iDynTree.Rotation();
    for i = 1 : suit.properties.nrOfLinks
        % get G_R_A matrix from quaternion data
        quaternion.fromMatlab(currentFrame.orientation((i-1)*4 + 1 : 4 * i));
        rotation.fromQuaternion(quaternion);
        G_R_A(:,:,i) = rotation.toMatlab;
        % compute T_R_A using Npose field
        quaternion.fromMatlab(suit.calibration.identity.orientation(:,i));
        rotation.fromQuaternion(quaternion);
        T_R_A(:,:,i) = rotation.toMatlab;
        % compute A_R_T
        A_R_T(:,:,i) = T_R_A(:,:,i)';
        % compute G_R_T
        G_Rot_T(:,:,i) = G_R_A(:,:,i) * A_R_T(:,:,i);
        % re-transform G_R_T in quaternion
        rotation.fromMatlab(G_Rot_T(:,:,i));
        G_q_T = rotation.asQuaternion();

% %         % ====test RPY
% %         if j == 4998  %Tpose position
% %         G_RPY_T(i,:) = rotation.asRPY.toMatlab() / pi * 180;
% %         else
% %             break
% %         end
% %         % ===========

        suit.links{i}.meas.orientation(:,j)         = G_q_T.toMatlab();
        suit.links{i}.meas.position(:,j)            = currentFrame.position(1, b*(i-1)+1 : b*i);
        suit.links{i}.meas.velocity(:,j)            = currentFrame.velocity(1, b*(i-1)+1 : b*i);
        suit.links{i}.meas.acceleration(:,j)        = currentFrame.acceleration(1, b*(i-1)+1 : b*i);
        suit.links{i}.meas.angularVelocity(:,j)     = currentFrame.angularVelocity(1, b*(i-1)+1 : b*i);
        suit.links{i}.meas.angularAcceleration(:,j) = currentFrame.angularAcceleration(1, b*(i-1)+1 : b*i); 
    end
    % --------JOINTS
    for i = 1 : suit.properties.nrOfJoints
        suit.joints{i}.meas.jointAngle(:,j)    = currentFrame.jointAngle(1, b*(i-1)+1 : b*i); 
        suit.joints{i}.meas.jointAngleXZY(:,j) = currentFrame.jointAngleXZY(1, b*(i-1)+1 : b*i); 
    end
    % --------SENSORS
    for i = 1 : suit.properties.nrOfSensors
        suit.sensors{i}.meas.sensorOrientation(:,j)     = currentFrame.sensorOrientation(1, a*(i-1)+1 : a*i);
          suit.sensors{i}.meas.sensorFreeAcceleration   = currentFrame.sensorFreeAcceleration(1, b*(i-1)+1 : b*i);
          suit.sensors{i}.meas.sensorMagneticField(:,j) = currentFrame.sensorMagneticField(1, b*(i-1)+1 : b*i);
    end
    j = j + 1;
end
