
% Insert here your code
% ....
% ....
% ....

%% Transform the sensorFreeAcceleration of MVNX2018 into the oldest version
% The following code is useful to transform the new MVN2018
% sensorFreeAcceleration into the oldest version.
if ~exist(fullfile(bucket.pathToProcessedData,'suit_runtime.mat'), 'file')
    gravity = [0; 0; -9.81];
    for sensIdx = 1: size(suit.sensors,1)
            len = size(suit_runtime.sensors{sensIdx, 1}.meas.sensorOrientation,2);
            for lenIdx = 1 : len
                G_R_S = quat2Mat(suit_runtime.sensors{sensIdx, 1}.meas.sensorOrientation(:,lenIdx));% fromQuaternion(quaternion);
                % Transformation:        S_a_old = S_R_G * (G_a_new - gravity)
                suit_runtime.sensors{sensIdx, 1}.meas.sensorOldAcceleration(:,lenIdx) = ...
                    transpose(G_R_S) * (suit_runtime.sensors{sensIdx, 1}.meas.sensorFreeAcceleration(:,lenIdx) - gravity);
            end
    end
    save(fullfile(bucket.pathToProcessedData,'suit_runtime.mat'),'suit_runtime');
else
    load(fullfile(bucket.pathToProcessedData,'suit_runtime.mat'));
end

% ....
% ....
% ....