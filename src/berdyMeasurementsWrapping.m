function [ y,Sigmay ] = berdyMeasurementsWrapping(data)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% 1 = ACCELEROMETER
% 2 = JOINT ACC
% 3 = EXTERNAL WRENCH
% 4 = GYROSCOPE

berdyInfo = [];

temp = struct;
temp.type = 1;
temp.id  = 'imu_acc';
berdyInfo = [berdyInfo; temp];

temp.type = 2;
temp.id  = 'ankle';
berdyInfo = [berdyInfo; temp];

temp.type = 2;
temp.id  = 'hip';
berdyInfo = [berdyInfo; temp];

temp.type = 3;
temp.id  = 'foot';
berdyInfo = [berdyInfo; temp];

temp.type = 3;
temp.id  = 'leg';
berdyInfo = [berdyInfo; temp];

temp.type = 3;
temp.id  = 'torso';
berdyInfo = [berdyInfo; temp];


    
y = [];
Sigmay = [];
for i = 1:length(berdyInfo)
    currentInfo = berdyInfo(i);
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
        error(strcat('Could not find sensor type', num2str(currentInfo.type), ' id: ', currentInfo.id ));
    end
end




end

