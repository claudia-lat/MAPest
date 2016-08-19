function [y, Sigmay] = berdyMeasurementsWrapping(berdy, data)
% BERDYMEASUREMENTSWRAPPING orders the measurements in a format
% compatible with the BerdyHelper class.  It returns a vector of
% ordered measurements and its associated covariance matrix.

sensorOrder = berdy.getSensorsOrdering();

y = [];
Sigmay = [];
for i = 1:size(sensorOrder,2)
    currentInfo = sensorOrder{i};
    found = false;
    for j = 1 : length(data)
        data_j = data(j);
        if (data_j.type == currentInfo.type && ...
            strcmp(data_j.id, currentInfo.id))
            found = true;
            y = [y; data_j.meas];
            Sigmay = blkdiag(Sigmay, diag(data_j.var));
            break;
        end
    end
    if (found == false)
        error(strcat('Could not find sensor type', num2str(currentInfo.type),...
             ' id: ', currentInfo.id ));
    end
end
end