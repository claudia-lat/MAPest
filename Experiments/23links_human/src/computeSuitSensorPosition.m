function [suit] = computeSuitSensorPosition(suit)
% COMPUTESUITSENSORPOSITION computes the position of the sensors in the
% suite wrt the link frame. It returns its value in a new field of the same
% suit stucture. Notation: G = global, S = sensor; L = link.

len = suit.properties.lenData;

for sIdx = 1: suit.properties.nrOfSensors
    sensor = suit.sensors{sIdx};
    [link, ~] = linksFromName(suit.links, sensor.attachedLink);
    A = zeros(3*len,3);
    b = zeros(3*len,1); 
    
    for i = 1 : len    
        S1 = skewMatrix(link.meas.angularAcceleration(:,i));
        S2 = skewMatrix(link.meas.angularVelocity(:,i)); 
        quaternion = iDynTree.Vector4();
        quaternion.fromMatlab(link.meas.orientation(:,i));
        G_R_L = iDynTree.Rotation();
        G_R_L.fromQuaternion(quaternion);
        G_R_L = G_R_L.toMatlab();
        A(3*i-2:3*i,:) = (S1 + S2*S2) * G_R_L;

        quaternion = iDynTree.Vector4();
        quaternion.fromMatlab(sensor.meas.sensorOrientation(:,i));
        G_R_S = iDynTree.Rotation();
        G_R_S.fromQuaternion(quaternion);
        G_R_S = G_R_S.toMatlab();
        G_acc_S = G_R_S * sensor.meas.sensorAcceleration(:,i);

        G_acc_L = link.meas.acceleration(:,i);

        b(3*i-2:3*i) = G_acc_S - G_acc_L;
    end
    % matrix system 
    B_pos_SL = A\b;
 
    sensor.origin = B_pos_SL; 
    suit.sensors{sIdx}.position = sensor.origin;   
end  
end

function [ S ] = skewMatrix(x)
%SKEWMATRIX computes the skew matrix given a vector x 3x1

S = [  0   -x(3)   x(2);
      x(3)   0    -x(1);
     -x(2)  x(1)    0  ];
end
