
suit.time = suit.time(:,1:cutIndex);
suit.COM = suit.COM(:, 1:cutIndex);
suit.properties.lenData = cutIndex;

for i = 1 : size(suit.links,1) 
    suit.links{i, 1}.meas.orientation = suit.links{i, 1}.meas.orientation(:,1:cutIndex);
    suit.links{i, 1}.meas.position = suit.links{i, 1}.meas.position(:,1:cutIndex);
    suit.links{i, 1}.meas.velocity = suit.links{i, 1}.meas.velocity(:,1:cutIndex);
    suit.links{i, 1}.meas.acceleration = suit.links{i, 1}.meas.acceleration(:,1:cutIndex);
    suit.links{i, 1}.meas.angularVelocity = suit.links{i, 1}.meas.angularVelocity(:,1:cutIndex);
    suit.links{i, 1}.meas.angularAcceleration = suit.links{i, 1}.meas.angularAcceleration(:,1:cutIndex);
end

for i = 1 : size(suit.joints,1) 
    suit.joints{1, 1}.meas.jointAngle = suit.joints{1, 1}.meas.jointAngle(:,1:cutIndex);
    suit.joints{1, 1}.meas.jointAngleXZY = suit.joints{1, 1}.meas.jointAngleXZY(:,1:cutIndex);
end

for i = 1 : size(suit.sensors,1) 
    suit.sensors{i, 1}.meas.sensorAcceleration_raw = suit.sensors{i, 1}.meas.sensorAcceleration_raw(:,1:cutIndex);
    suit.sensors{i, 1}.meas.sensorAcceleration = suit.sensors{i, 1}.meas.sensorAcceleration(:,1:cutIndex);
    suit.sensors{i, 1}.meas.sensorAngularVelocity = suit.sensors{i, 1}.meas.sensorAngularVelocity(:,1:cutIndex);
    suit.sensors{i, 1}.meas.sensorOrientation = suit.sensors{i, 1}.meas.sensorOrientation(:,1:cutIndex);
    suit.sensors{i, 1}.meas.sensorMagneticField = suit.sensors{i, 1}.meas.sensorMagneticField(:,1:cutIndex);
end