function [suitStruct] = dataSync(suitStruct, syncIndex)
% DATASYNC: function to  synchronized all the dataset of the experiment 
% (suit, forceplate and robot).  

% Inputs 
% -  suitStruct       : suit data;
% -  forceplateStruct : forceplate data;
% -  syncIndex        : index of the suit data that correspond to the robot
%                       data;
% -  suitTimeInit     : initial unix time for the suit data.

% Outputs
% -  suitStruct       : suit data synchronized with robot data;
% -  forceplateStruct : forceplate data synchronized with robot data;
% -  suitSyncIndex    : index used for synnchronized the suit data with the
%                       final dataset.

%% Create data struct for the suit
% PROPERTIES
nrOfFrames = size(syncIndex,1);
suitStruct.properties.lenData = nrOfFrames;
% TIME
suitStruct.time = suitStruct.time(:,syncIndex);
% COM
suitStruct.COM = suitStruct.COM(:,syncIndex);
% LINKS
for i = 1 : suitStruct.properties.nrOfLinks
    suitStruct.links{i}.meas.orientation = suitStruct.links{i}.meas.orientation(:,syncIndex); 
    suitStruct.links{i}.meas.position = suitStruct.links{i}.meas.position(:,syncIndex);
    suitStruct.links{i}.meas.velocity = suitStruct.links{i}.meas.velocity(:,syncIndex);
    suitStruct.links{i}.meas.acceleration = suitStruct.links{i}.meas.acceleration(:,syncIndex);
    suitStruct.links{i}.meas.angularVelocity = suitStruct.links{i}.meas.angularVelocity(:,syncIndex);
    suitStruct.links{i}.meas.angularAcceleration = suitStruct.links{i}.meas.angularAcceleration(:,syncIndex); 
end
% JOINTS
for i = 1 : suitStruct.properties.nrOfJoints
    suitStruct.joints{i}.meas.jointAngle = suitStruct.joints{i}.meas.jointAngle(:,syncIndex); 
    suitStruct.joints{i}.meas.jointAngleXZY = suitStruct.joints{i}.meas.jointAngleXZY(:,syncIndex); 
end
% SENSORS
for i = 1 : suitStruct.properties.nrOfSensors
    suitStruct.sensors{i}.meas.sensorAcceleration = suitStruct.sensors{i}.meas.sensorAcceleration(:,syncIndex); 
    suitStruct.sensors{i}.meas.sensorAngularVelocity = suitStruct.sensors{i}.meas.sensorAngularVelocity(:,syncIndex); 
    suitStruct.sensors{i}.meas.sensorMagneticField = suitStruct.sensors{i}.meas.sensorMagneticField(:,syncIndex); 
    suitStruct.sensors{i}.meas.sensorOrientation = suitStruct.sensors{i}.meas.sensorOrientation(:,syncIndex); 
end

end
