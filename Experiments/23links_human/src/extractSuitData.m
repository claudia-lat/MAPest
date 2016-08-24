function [ suit ] = extractSuitData(mvnxFilename, outputDir)
%EXTRACTSUITDATA allows to create a .mat stucture contatining all suit data 
% acquired during the Xsens experiment. Since Xsens provides the data in a 
% .mvnx file, you need the following dependency to run it:
% /MAPest/external/xml_io_tools.
%
% Inputs 
% -  mvnxFilename : the path to the file.mvnx;
% -  outputDir    : (optional) the directory where saving the output.
% Outputs
% -  suit         : data of the acquisition in a .mat format. 


%% Load and read file .mvnx
 addpath(genpath('../../external'));
 mvnxData = xml_read(mvnxFilename);
%% Create data struct
suit =[];
% PROPERTIES
suit.properties.experimentLabel = mvnxData.subject.ATTRIBUTE.label;
suit.properties.recordingDate   = mvnxData.subject.ATTRIBUTE.recDate;
suit.properties.frameRate       = mvnxData.subject.ATTRIBUTE.frameRate;
nrOfFrames                      = size(mvnxData.subject.frames.frame,1);
suit.properties.nrOfLinks       = mvnxData.subject.frames.ATTRIBUTE.segmentCount;
suit.properties.nrOfJoints      = mvnxData.subject.frames.ATTRIBUTE.jointCount;
suit.properties.nrOfSensors     = mvnxData.subject.frames.ATTRIBUTE.sensorCount;
suit.properties.lenData         = nrOfFrames;
% CALIBRATION 
suit.calibration = struct;
counterCalibr = 0;
for j = 1 : suit.properties.lenData
    if (strcmp(mvnxData.subject.frames.frame(j).ATTRIBUTE.type , 'npose'))
        counterCalibr = counterCalibr + 1;
    end
    if (strcmp(mvnxData.subject.frames.frame(j).ATTRIBUTE.type , 'tpose'))
        counterCalibr = counterCalibr + 1;
    end
end
suit.properties.lenData = suit.properties.lenData - counterCalibr;
% TIME
suit.time = zeros(1,suit.properties.lenData);
% COM
suit.COM  = zeros(3,suit.properties.lenData);
% LINKS
suit.links = cell(suit.properties.nrOfLinks, 1);
for i = 1 : suit.properties.nrOfLinks
    suit.links{i}.id = mvnxData.subject.segments.segment(i).ATTRIBUTE.id;
    suit.links{i}.label = mvnxData.subject.segments.segment(i).ATTRIBUTE.label;
    suit.links{i}.meas = struct;
    suit.links{i}.meas.orientation         = zeros(4, suit.properties.lenData);
    suit.links{i}.meas.position            = zeros(3, suit.properties.lenData);
    suit.links{i}.meas.velocity            = zeros(3, suit.properties.lenData);
    suit.links{i}.meas.acceleration        = zeros(3, suit.properties.lenData);
    suit.links{i}.meas.angularVelocity     = zeros(3, suit.properties.lenData);
    suit.links{i}.meas.angularAcceleration = zeros(3, suit.properties.lenData);
    suit.links{i}.points                   = struct;
    suit.links{i}.points.nrOfPoints        = size(mvnxData.subject.segments.segment(i).points.point,1); 
    suit.links{i}.points.pointsValue       = zeros(3,suit.links{i}.points.nrOfPoints);
    for k = 1 : suit.links{i}.points.nrOfPoints
        suit.links{i}.points.label(1,k) = cellstr(mvnxData.subject.segments.segment(i).points.point(k).ATTRIBUTE.label);
        suit.links{i}.points.pointsValue(:,k) = mvnxData.subject.segments.segment(i).points.point(k).pos_s;
    end
end
% JOINTS
suit.joints = cell(suit.properties.nrOfJoints,1);
for i = 1 : suit.properties.nrOfJoints
    suit.joints{i}.label              = mvnxData.subject.joints.joint(i).ATTRIBUTE.label;
    suit.joints{i}.meas               = struct;
    suit.joints{i}.meas.jointAngle    = zeros(3, suit.properties.lenData);
    suit.joints{i}.meas.jointAngleXZY = zeros(3, suit.properties.lenData);
end
% SENSORS
suit.sensors = cell(suit.properties.nrOfSensors,1);
for i = 1 : suit.properties.nrOfSensors
    suit.sensors{i}.label                      = mvnxData.subject.sensors.sensor(i).ATTRIBUTE;
    suit.sensors{i}.meas.sensorAcceleration    = zeros(3, suit.properties.lenData);
    suit.sensors{i}.meas.sensorAngularVelocity = zeros(3, suit.properties.lenData);
    suit.sensors{i}.meas.sensorOrientation     = zeros(4, suit.properties.lenData);
end 
%% Fill the struct with recording data
a = 4; %dimension of quaternions
b = 3; %dimension of vectors
j = 1;
for frameIdx = 1 : nrOfFrames
    
    currentFrame = mvnxData.subject.frames.frame(frameIdx);
    
    if (strcmp(mvnxData.subject.frames.frame(frameIdx).ATTRIBUTE.type, 'npose'))
        suit.calibration.npose             = struct;
        suit.calibration.npose.index       = currentFrame.ATTRIBUTE.index;
        suit.calibration.npose.orientation = zeros(4, suit.properties.nrOfLinks);
        suit.calibration.npose.position    = zeros(3, suit.properties.nrOfLinks);
        for i = 1 : suit.properties.nrOfLinks
            suit.calibration.npose.orientation(:,i) = currentFrame.orientation(1, a*(i-1)+1 : a*i);
            suit.calibration.npose.position(:,i)    = currentFrame.position(1, b*(i-1)+1 : b*i);
        end
        continue;
    end
    if (strcmp(mvnxData.subject.frames.frame(frameIdx).ATTRIBUTE.type, 'tpose'))
        suit.calibration.tpose             = struct;
        suit.calibration.tpose.index       = currentFrame.ATTRIBUTE.index;
        suit.calibration.tpose.orientation = zeros(4, suit.properties.nrOfLinks);
        suit.calibration.tpose.position    = zeros(3, suit.properties.nrOfLinks);
        for i = 1 : suit.properties.nrOfLinks
            suit.calibration.tpose.orientation(:,i) = currentFrame.orientation(1, a*(i-1)+1 : a*i);
            suit.calibration.tpose.position(:,i)    = currentFrame.position(1, b*(i-1)+1 : b*i);
        end
        continue;
    end       
    % /The frame from now on is neither 'tpose' nor 'npose', thus
    % it is normal./
    % TIME
    suit.time(1,j) = currentFrame.ATTRIBUTE.ms;
    % COM
    suit.COM(:,j) = currentFrame.centerOfMass';
    % LINKS
    for i = 1 : suit.properties.nrOfLinks
        suit.links{i}.meas.orientation(:,j)         = currentFrame.orientation(1, a*(i-1)+1 : a*i); 
        suit.links{i}.meas.position(:,j)            = currentFrame.position(1, b*(i-1)+1 : b*i);
        suit.links{i}.meas.velocity(:,j)            = currentFrame.velocity(1, b*(i-1)+1 : b*i);
        suit.links{i}.meas.acceleration(:,j)        = currentFrame.acceleration(1, b*(i-1)+1 : b*i);
        suit.links{i}.meas.angularVelocity(:,j)     = currentFrame.angularVelocity(1, b*(i-1)+1 : b*i);
        suit.links{i}.meas.angularAcceleration(:,j) = currentFrame.angularAcceleration(1, b*(i-1)+1 : b*i); 
    end
    % JOINTS
    for i = 1 : suit.properties.nrOfJoints
        suit.joints{i}.meas.jointAngle(:,j)    = currentFrame.jointAngle(1, b*(i-1)+1 : b*i); 
        suit.joints{i}.meas.jointAngleXZY(:,j) = currentFrame.jointAngleXZY(1, b*(i-1)+1 : b*i); 
    end
    % SENSORS
    for i = 1 : suit.properties.nrOfSensors
        suit.sensors{i}.meas.sensorAcceleration(:,j)    = currentFrame.sensorAcceleration(1, b*(i-1)+1 : b*i); 
        suit.sensors{i}.meas.sensorAngularVelocity(:,j) = currentFrame.sensorAngularVelocity(1, b*(i-1)+1 : b*i); 
        suit.sensors{i}.meas.sensorMagneticField(:,j)   = currentFrame.sensorMagneticField(1, b*(i-1)+1 : b*i); 
        suit.sensors{i}.meas.sensorOrientation(:,j)     = currentFrame.sensorOrientation(1, a*(i-1)+1 : a*i); 
    end 
    j = j + 1;
end
%% Save data in a file.mat
if nargin == 2
    filename = sprintf('%s_suit.mat',strrep(strtrim(suit.properties.experimentLabel),' ','_'));
    if ~exist(outputDir,'dir')
        mkdir(outputDir);
    end
    save(fullfile(outputDir, filename),'suit');
end
rmpath(genpath('../../external'));
end
