function [suitStruct, forceplateStruct,  suitSyncIndex] = dataSync(suitStruct, forceplateStruct, syncIndex, suitIndex)
% DATASYNC synchronizes all the dataset of the experiment 
% (suit, forceplate and robot).  
%
% Inputs 
% -  suitStruct       : suit data;
% -  forceplateStruct : forceplate data;
% -  syncIndex        : index of the suit data that correspond to the robot
%                       data;
% -  suitTimeInit     : initial unix time for the suit data.
%
% Outputs
% -  suitStruct       : suit data synchronized with robot data;
% -  forceplateStruct : forceplate data synchronized with robot data;
% -  suitSyncIndex    : index used for synnchronized the suit data with the
%                       final dataset.

%% Create data struct for the forceplate
% PROPERTIES
nrOfFrames = size(syncIndex,1);
forceplateStruct.data.properties.nrOfFrame = nrOfFrames;
% TIME
forceplateStruct.data.time.unixTime = forceplateStruct.data.time.unixTime(1,syncIndex);
%forceplateStruct.data.time.standardTime = forceplateStruct.data.time.standardTime(1,syncIndex);
% PLATEFORMS
forceplateStruct.data.plateforms.plateform1.frames = forceplateStruct.data.plateforms.plateform1.frames(1,syncIndex);
forceplateStruct.data.plateforms.plateform1.forces = forceplateStruct.data.plateforms.plateform1.forces(:,syncIndex);
forceplateStruct.data.plateforms.plateform1.moments = forceplateStruct.data.plateforms.plateform1.moments(:,syncIndex) ;
forceplateStruct.data.plateforms.plateform2.frames = forceplateStruct.data.plateforms.plateform2.frames(1,syncIndex);
forceplateStruct.data.plateforms.plateform2.forces = forceplateStruct.data.plateforms.plateform2.forces(:,syncIndex);
forceplateStruct.data.plateforms.plateform2.moments = forceplateStruct.data.plateforms.plateform2.moments(:,syncIndex);

%% Create data struct for the suit
suitIndex = suitIndex(syncIndex');
% PROPERTIES
nrOfFrames = size(suitIndex,1);
suitStruct.properties.lenData = nrOfFrames;
% TIME
suitStruct.time = forceplateStruct.data.time.unixTime;
% COM
suitStruct.COM = suitStruct.COM(:,suitIndex);
% LINKS
for i = 1 : suitStruct.properties.nrOfLinks
    suitStruct.links{i}.meas.orientation = suitStruct.links{i}.meas.orientation(:,suitIndex); 
    suitStruct.links{i}.meas.position = suitStruct.links{i}.meas.position(:,suitIndex);
    suitStruct.links{i}.meas.velocity = suitStruct.links{i}.meas.velocity(:,suitIndex);
    suitStruct.links{i}.meas.acceleration = suitStruct.links{i}.meas.acceleration(:,suitIndex);
    suitStruct.links{i}.meas.angularVelocity = suitStruct.links{i}.meas.angularVelocity(:,suitIndex);
    suitStruct.links{i}.meas.angularAcceleration = suitStruct.links{i}.meas.angularAcceleration(:,suitIndex); 
end
% JOINTS
for i = 1 : suitStruct.properties.nrOfJoints
    suitStruct.joints{i}.meas.jointAngle = suitStruct.joints{i}.meas.jointAngle(:,suitIndex); 
    suitStruct.joints{i}.meas.jointAngleXZY = suitStruct.joints{i}.meas.jointAngleXZY(:,suitIndex); 
end
% SENSORS
for i = 1 : suitStruct.properties.nrOfSensors
    suitStruct.sensors{i}.meas.sensorAcceleration = suitStruct.sensors{i}.meas.sensorAcceleration(:,suitIndex); 
    suitStruct.sensors{i}.meas.sensorAngularVelocity = suitStruct.sensors{i}.meas.sensorAngularVelocity(:,suitIndex); 
    suitStruct.sensors{i}.meas.sensorMagneticField = suitStruct.sensors{i}.meas.sensorMagneticField(:,suitIndex); 
    suitStruct.sensors{i}.meas.sensorOrientation = suitStruct.sensors{i}.meas.sensorOrientation(:,suitIndex); 
end

%% Determine the final synchronization index for xsens data
suitSyncIndex = suitIndex; % index for synchronizing 
                                                              % the original data of the suit (mvnx file)
end
