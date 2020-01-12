function [ index, len ] = rangeOfSensorMeasurement( berdy, sensType, sensID, stackOfTaskMAP)
%RANGEOFDYNAMICVARIABLE given a type of sensor and its label, returns its
% index in the vector y and its range.

sensorOrder = berdy.getSensorsOrdering(stackOfTaskMAP);
index = -1;
len = 0;
for i = 1:size(sensorOrder,2)
    currentInfo = sensorOrder{i};
    if currentInfo.type == sensType && strcmp(currentInfo.id, sensID)
        range = currentInfo.range;
        index = range.offset + 1;
        len = range.size;
        break;
    end
end
end