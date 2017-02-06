function [ dataPacked ] = dataPackaging(model, sensors, suit, forceplate, ddq, robot)
%DATAPACKAGING creates a data struct organised in the following way:
% - data.time (a unified time for all type of sensors)
% Each substructure is identified by:
%    - type: the type associated to the sensors in iDynTree;
%    - id  : labels coming from 'model' and 'sensor';
%    - meas: value of measurements;
%    - var : variance of sensor.

data      = struct;
data.acc  = struct;
% data.gyro = struct;
data.ddq  = struct;       
data.fext = struct;

%% FROM SUIT
% SENSOR: <ACCELEROMETER>
% type
data.acc.type = iDynTree.ACCELEROMETER_SENSOR;
% id
nOfSensor.acc = sensors.getNrOfSensors(data.acc.type);
data.acc.id   = cell(nOfSensor.acc,1);
for i = 1 : nOfSensor.acc 
   data.acc.id{i} = sensors.getSensor(data.acc.type, i-1).getName;
end
% meas
data.acc.meas = cell(nOfSensor.acc,1);
tempData = cell(nOfSensor.acc,2);
sensorsLabelToCmp  = cell(nOfSensor.acc,1);
nOfSensorsFromSuit = size(suit.sensors,1);
for i = 1 :  nOfSensor.acc
    tempData(i,:) = strsplit(data.acc.id{i}, '_');
    sensorsLabelToCmp{i}= tempData(i,1);
    for j = 1 : nOfSensorsFromSuit 
        if  strcmp(sensorsLabelToCmp{i},suit.sensors{j, 1}.label)
            data.acc.meas{i} = suit.sensors{i, 1}.meas.sensorAcceleration;
            break;
        end
    end
end
% variance
data.acc.var     = 0.001111 * ones(3,1); %from datasheet

% % % SENSOR: <GYROSCOPE>
% % % type
% % data.gyro.type = iDynTree.GYROSCOPE_SENSOR;
% % % id
% % nOfSensor.gyro = sensors.getNrOfSensors(data.gyro.type);
% % data.gyro.id = cell(nOfSensor.gyro,1);
% % for i = 1 : nOfSensor.gyro
% %    data.gyro.id{i} = sensors.getSensor(data.gyro.type, i-1).getName;
% % end
% % % meas
% % data.gyro.meas = cell(nOfSensor.gyro,1);
% % tempData = cell(nOfSensor.gyro,2);
% % sensorsLabelToCmp  = cell(nOfSensor.gyro,1);
% % nOfSensorsFromSuit = size(suit.sensors,1);
% % for i = 1 :  nOfSensor.gyro
% %     tempData(i,:) = strsplit(data.gyro.id{i}, '_');
% %     sensorsLabelToCmp{i}= tempData(i,1);
% %     for j = 1 : nOfSensorsFromSuit
% %         if  strcmp(sensorsLabelToCmp{i},suit.sensors{j, 1}.label)
% %             data.gyro.meas{i} = suit.sensors{i, 1}.meas.sensorAngularVelocity;
% %             break;
% %         end
% %     end
% % end
% % % variance
% % data.gyro.var = 0.001111 * ones(3,1); %from datasheet
%
%% FROM ddq
nOfSensor.DOFacc = size(ddq,1);
jointNameFromModel = cell(nOfSensor.DOFacc,1);
for i = 1 : nOfSensor.DOFacc 
    jointNameFromModel{i} = model.getJointName(i-1);
end

% SENSOR: <DOF ACCELERATION SENSOR>
% type
data.ddq.type = iDynTree.DOF_ACCELERATION_SENSOR;
% id & meas
data.ddq.id   = cell(size(jointNameFromModel));
data.ddq.meas = cell(size(jointNameFromModel));
for i = 1 : nOfSensor.DOFacc
    data.ddq.id{i}   = jointNameFromModel{i};
    data.ddq.meas{i} = ddq(i,:); 
end
% variance
data.ddq.var = 6.66e-6; %from datasheet

%% FROM FORCEPLATE & ROBOT
% ---------------------------------------------------------------
% Both sensors are considered as external force acting as follow:
% - on human rightFoot --> force platform1;
% - on human leftFoot  --> force platform2;
% - on human rightHand --> robot left arm;
% - on human leftHand  --> robot right arm;
% - null meas for all the others.
% ---------------------------------------------------------------
nOfSensor.fext = model.getNrOfLinks;
linkNameFromModel = cell(model.getNrOfLinks,1);
for i = 1 : model.getNrOfLinks 
    linkNameFromModel{i} = model.getLinkName(i-1);
end

% SENSOR: <EXTERNAL FORCES> 
% type  
data.fext.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% id 
data.fext.id   = cell(size(linkNameFromModel));
for i = 1 : nOfSensor.fext
    data.fext.id{i}     = linkNameFromModel{i};
end
% meas
data.fext.meas = cell(size(linkNameFromModel));
% get the contactLink
contactLink = cell(2,1);
contactLink{1} = forceplate.data.plateforms.plateform1.contactLink;
contactLink{2} = forceplate.data.plateforms.plateform2.contactLink;
contactLink{3} = robot.data.links.rightarm.contactLink;
contactLink{4} = robot.data.links.leftarm.contactLink;
index = cell(size(contactLink));
for i = 1: size(contactLink,1)
    for indx = 1 : nOfSensor.fext
        if  strcmp(data.fext.id{indx},contactLink{i})
            index{i} = indx;
        end
    end
end
% fill the all meas with null fext
wrench = zeros(6,size(forceplate.processedData.humanLeftFootWrench,2));
for i =  1 : nOfSensor.fext
    data.fext.meas{i} = wrench;
    % fill with the 4 fext that are not null
    % <FOR FORCEPLATE>
    if i == index{1}
        data.fext.meas{i} = forceplate.processedData.humanRightFootWrench;
    elseif i == index{2}
        data.fext.meas{i} = forceplate.processedData.humanLeftFootWrench;
    % <FOR ROBOT>
    elseif i == index{3}
        data.fext.meas{i} = robot.processedData.humanLeftHandWrench;
    elseif i == index{4}
        data.fext.meas{i} = robot.processedData.humanRightHandWrench;
    end
end
% variance
 data.fext.var = 1e-6 * ones(6,1); % variance for f ext 
%% Final Packaging
dataPacked = struct;
for i = 1 : nOfSensor.acc
    dataPacked(i).type                  = data.acc.type;
    dataPacked(i).id                    = data.acc.id{i};
    dataPacked(i).meas                  = data.acc.meas{i};
    dataPacked(i).var                   = data.acc.var;
%     dataPacked(i + nOfSensor.acc).type  = data.gyro.type;
%     dataPacked(i + nOfSensor.acc).id    = data.gyro.id{i};
%     dataPacked(i + nOfSensor.acc).meas  = data.gyro.meas{i};
%     dataPacked(i + nOfSensor.acc).var   = data.gyro.var;
end
indx = nOfSensor.acc;
% indx = i;
%--
for i = 1 : nOfSensor.DOFacc
    dataPacked(i + (indx)).type         = data.ddq.type;
    dataPacked(i + (indx)).id           = data.ddq.id{i};
    dataPacked(i + (indx)).meas         = data.ddq.meas{i};
    dataPacked(i + (indx)).var          = data.ddq.var;
end
 indx = indx + nOfSensor.DOFacc;

for i = 1 : nOfSensor.fext
    dataPacked(i + (indx)).type         = data.fext.type;
    dataPacked(i + (indx)).id           = data.fext.id{i};
    dataPacked(i + (indx)).meas         = data.fext.meas{i};
    dataPacked(i + (indx)).var          = data.fext.var;
    if i == index{1}
         dataPacked(i + (indx)).var     = 1e-3 * [59; 59; 36; 2.25; 2.25; 0.56]; %from datasheet
    elseif i == index{2}
        dataPacked(i + (indx)).var      = 1e-3 * [59; 59; 36; 2.25; 2.25; 0.56]; %from datasheet
    % <FOR ROBOT>
    elseif i == index{3}
        dataPacked(i + (indx)).var      = 1e-3 * [59; 59; 36; 2.25; 2.25; 0.56]; %from datasheet
    elseif i == index{4}
        dataPacked(i + (indx)).var      = 1e-3 * [59; 59; 36; 2.25; 2.25; 0.56]; %from datasheet
    end
end
end
