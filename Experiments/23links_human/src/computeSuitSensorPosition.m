function [suit] = computeSuitSensorPosition(suit)
% COMPUTESUITSENSORPOSITION computes the position of the sensors in the
% suit wrt the link frame. It returns its value in a new field of the same
% suit stucture. Notation: G = global, S = sensor; L = link.

% len = suit.properties.lenData;
len = 10000; % fixed nr of frames useful to capture all the movements done
             % during the experiment.

quaternion = iDynTree.Vector4();
G_R_L = iDynTree.Rotation();
G_R_S = iDynTree.Rotation();
rot = iDynTree.Rotation();

for sIdx = 1: suit.properties.nrOfSensors
    sensor = suit.sensors{sIdx};
    [link, ~] = linksFromName(suit.links, sensor.attachedLink);
    A = zeros(3*len,3);
    b = zeros(3*len,1);

    for i = 1 : len
        S1 = skewMatrix(link.meas.angularAcceleration(:,i));
        S2 = skewMatrix(link.meas.angularVelocity(:,i));

        quaternion.fromMatlab(link.meas.orientation(:,i));
        G_R_L.fromQuaternion(quaternion);
        G_R_L_mat = G_R_L.toMatlab();
        A(3*i-2:3*i,:) = (S1 + S2*S2) * G_R_L_mat;

        G_acc_S = sensor.meas.sensorFreeAcceleration(:,i);
        G_acc_L = link.meas.acceleration(:,i);

        b(3*i-2:3*i) = G_acc_S - G_acc_L;

        % compute S_R_L = S_R_G x G_R_L
        quaternion.fromMatlab(sensor.meas.sensorOrientation(:,i));
        G_R_S.fromQuaternion(quaternion);
        G_R_S_mat = G_R_S.toMatlab();
        S_R_L = G_R_S_mat' * G_R_L_mat;
        L_R_S = S_R_L' ;

        rot.fromMatlab(L_R_S);
        L_RPY_S(i,:) = rot.asRPY.toMatlab(); %RPY in rad
    end
    % matrix system
    B_pos_SL = A\b;
    sensor.origin = B_pos_SL;
    suit.sensors{sIdx}.position = sensor.origin;
    suit.sensors{sIdx}.RPY = mean(L_RPY_S);

end
end

function [ S ] = skewMatrix(x)
%SKEWMATRIX computes the skew matrix given a vector x 3x1

S = [  0   -x(3)   x(2);
    x(3)   0    -x(1);
    -x(2)  x(1)    0  ];
end
