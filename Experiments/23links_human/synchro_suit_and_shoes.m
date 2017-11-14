%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  THIS SCRIPT IS FOR THE SENSOR COMBINATION Xsens + ftShoes  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Load shoes measurements YARP files 
shoes = struct;

% LEFT---------------------------------------------------------------------    
bucket.leftShoe_frontForce = sprintf(fullfile(bucket.pathToTrial, ...
    '/ftShoeDriver_Left/frontForce_000%02d/data.log'), trialID);
bucket.leftShoe_rearForce  = sprintf(fullfile(bucket.pathToTrial,...
    '/ftShoeDriver_Left/rearForce_000%02d/data.log'), trialID);
bucket.leftShoe_totalForce = sprintf(fullfile(bucket.pathToTrial,...
    '/ftShoeDriver_Left/totalForce_000%02d/data.log'), trialID);

% RIGHT--------------------------------------------------------------------
bucket.rightShoe_frontForce = sprintf(fullfile(bucket.pathToTrial,...
    '/ftShoeDriver_Right/frontForce_000%02d/data.log'), trialID);
bucket.rightShoe_rearForce  = sprintf(fullfile(bucket.pathToTrial,...
    '/ftShoeDriver_Right/rearForce_000%02d/data.log'), trialID);
bucket.rightShoe_totalForce = sprintf(fullfile(bucket.pathToTrial,...
    '/ftShoeDriver_Right/totalForce_000%02d/data.log'), trialID);

%% Parse shoes measurements
% LEFT---------------------------------------------------------------------
shoes.Left.frameRate  = 100; %100Hz
shoes.Left.frontForce = parseYARPftShoes_fromDriver(bucket.leftShoe_frontForce);
shoes.Left.rearForce  = parseYARPftShoes_fromDriver(bucket.leftShoe_rearForce);
shoes.Left.totalForce = parseYARPftShoes_fromDriver(bucket.leftShoe_totalForce);

% RIGHT--------------------------------------------------------------------
shoes.Right.frameRate  = 100; %100Hz
shoes.Right.frontForce = parseYARPftShoes_fromDriver(bucket.rightShoe_frontForce);
shoes.Right.rearForce  = parseYARPftShoes_fromDriver(bucket.rightShoe_rearForce);
shoes.Right.totalForce = parseYARPftShoes_fromDriver(bucket.rightShoe_totalForce); 

%% Synchronize YARP data (Front,Rear,Total) within each shoe
shoes.Left.single_synch  = synchroIndividualShoe(shoes.Left);
shoes.Right.single_synch = synchroIndividualShoe(shoes.Right); 

%% Synchronize measurements between Left and Right shoes
shoes = synchroShoes(shoes);

%% Synchronize shoes and suit  
% At this stage:
% - data from shoes are acquired at 100Hz, data from Xsens at 240Hz
% - suit acquisition started before shoes and ended later as in the
%   following scheme: (suit-----(shoes---------------------shoes)-----suit)

% -------------------------------------------------------------------------
% Alignment of the 2 modalities by maintaining their own acquisition freq
% -------------------------------------------------------------------------
% Define a vector suitIndex that gives the information on the index where
% the shoes signal starts and ends.  This will be useful for cutting the
% suit later.

suitIndex  = zeros(size(suit.time)); % for sure suitIndex won't be higher 
% than this dimension. I don't know what is its dimension now!
slaveTime = shoes.Right.interp_synch.time;
masterTime = suit.time .* 1.e-3; %to ms
admissibleTreshold = 1e-2; %minimum treshold for valid timestamp difference
for i = 1 : size(slaveTime,2)
    for j = 1: size(suit.time,2)
        if abs(slaveTime(:,i) - masterTime(:,j)) < admissibleTreshold
            suitIndex(i) = j;
            break
        end
    end
end

% Define a range for cutting
for i = 1 : size(suitIndex,2)
    if suitIndex(i) == 0
       bucket.firstZero = i;
       break
    end
end
rangeCut = (suitIndex(1):suitIndex(bucket.firstZero-1));

% Cut all the suit signals w.r.t. the suitIndex
% TIME
suit.time = suit.time(rangeCut);
% PROPERTIES
suit.properties.lenData = size(suit.time,2);
% COM
suit.COM = suit.COM(rangeCut);
% LINKS
for i = 1 : suit.properties.nrOfLinks
    suit.links{i}.meas.orientation = suit.links{i}.meas.orientation(:,rangeCut); 
    suit.links{i}.meas.position = suit.links{i}.meas.position(:,rangeCut);
    suit.links{i}.meas.velocity = suit.links{i}.meas.velocity(:,rangeCut);
    suit.links{i}.meas.acceleration = suit.links{i}.meas.acceleration(:,rangeCut);
    suit.links{i}.meas.angularVelocity = suit.links{i}.meas.angularVelocity(:,rangeCut);
    suit.links{i}.meas.angularAcceleration = suit.links{i}.meas.angularAcceleration(:,rangeCut); 
end
% JOINTS
for i = 1 : suit.properties.nrOfJoints
    suit.joints{i}.meas.jointAngle = suit.joints{i}.meas.jointAngle(:,rangeCut); 
    suit.joints{i}.meas.jointAngleXZY = suit.joints{i}.meas.jointAngleXZY(:,rangeCut); 
end
% SENSORS
for i = 1 : suit.properties.nrOfSensors
    suit.sensors{i}.meas.sensorAcceleration = suit.sensors{i}.meas.sensorAcceleration(:,rangeCut); 
    suit.sensors{i}.meas.sensorAngularVelocity = suit.sensors{i}.meas.sensorAngularVelocity(:,rangeCut); 
%     suit.sensors{i}.meas.sensorMagneticField = suit.sensors{i}.meas.sensorMagneticField(:,rangeCut);
    suit.sensors{i}.meas.sensorOrientation = suit.sensors{i}.meas.sensorOrientation(:,rangeCut); 
end

% -------------------------------------------------------------------------
% Upsampling at 240Hz
% -------------------------------------------------------------------------
% At this point I would have preferred to do a downsampling of the suit 
% (from 240Hz to 100Hz).  Instead here it is performed an upsampling 
% because for computing the IK later in the main, I need the .trc obtained 
% (through Mokka) from the .c3d obtained, in turn, (through Xsens exporter)
% from the original acquisition on which I can't do any action of cutting!


vectDim = size(shoes.Right.single_synch.totalForce.forces,1);

shoes.Left.upsampled.time = interp1(slaveTime, ...
                            shoes.Right.interp_synch.time', ...
                            masterTime);
shoes.Right.upsampled.time = shoes.Left.upsampled.time; %the time was already uniform!
for i = 1 : vectDim
    % LEFT-----------------------------------------------------------------    
    shoes.Left.upsampled.frontForce.forces(i,:) = interp1(slaveTime, ...
                             shoes.Left.interp_synch.frontForce.forces(i,:), ...
                             masterTime); 
    shoes.Left.upsampled.frontForce.moments(i,:) = interp1(slaveTime, ...
                             shoes.Left.interp_synch.frontForce.moments(i,:), ...
                             masterTime);
    %
    shoes.Left.upsampled.rearForce.forces(i,:) = interp1(slaveTime, ...
                             shoes.Left.interp_synch.rearForce.forces(i,:), ...
                             masterTime); 
    shoes.Left.upsampled.rearForce.moments(i,:) = interp1(slaveTime, ...
                             shoes.Left.interp_synch.rearForce.moments(i,:), ...
                             masterTime); 
    %
    shoes.Left.upsampled.totalForce.forces(i,:) = interp1(slaveTime, ...
                             shoes.Left.interp_synch.totalForce.forces(i,:), ...
                             masterTime);
    shoes.Left.upsampled.totalForce.moments(i,:) = interp1(slaveTime, ...
                             shoes.Left.interp_synch.totalForce.moments(i,:), ...
                             masterTime); 
    % RIGHT----------------------------------------------------------------
    shoes.Right.upsampled.frontForce.forces(i,:) = interp1(slaveTime, ...
                             shoes.Right.interp_synch.frontForce.forces(i,:), ...
                             masterTime); 
    shoes.Right.upsampled.frontForce.moments(i,:) = interp1(slaveTime, ...
                             shoes.Right.interp_synch.frontForce.moments(i,:), ...
                             masterTime); 
    %                     
    shoes.Right.upsampled.rearForce.forces(i,:) = interp1(slaveTime, ...
                             shoes.Right.interp_synch.rearForce.forces(i,:), ...
                             masterTime);
    shoes.Right.upsampled.rearForce.moments(i,:) = interp1(slaveTime, ...
                             shoes.Right.interp_synch.rearForce.moments(i,:), ...
                             masterTime);
    %
    shoes.Right.upsampled.totalForce.forces(i,:) = interp1(slaveTime, ...
                             shoes.Right.interp_synch.totalForce.forces(i,:), ...
                             masterTime); 
    shoes.Right.upsampled.totalForce.moments(i,:) = interp1(slaveTime, ...
                             shoes.Right.interp_synch.totalForce.moments(i,:), ...
                             masterTime); 
end

% Remove from signal NaN elements by applying rangeCut
% LEFT----------------------------------------------------------------- 
shoes.Left.upsampled.time = shoes.Left.upsampled.time(:,rangeCut);
%
shoes.Left.upsampled.frontForce.forces  = shoes.Left.upsampled.frontForce.forces(:,rangeCut); 
shoes.Left.upsampled.frontForce.moments = shoes.Left.upsampled.frontForce.moments(:,rangeCut);
%
shoes.Left.upsampled.rearForce.forces  = shoes.Left.upsampled.rearForce.forces(:,rangeCut); 
shoes.Left.upsampled.rearForce.moments = shoes.Left.upsampled.rearForce.moments(:,rangeCut);
%
shoes.Left.upsampled.totalForce.forces  = shoes.Left.upsampled.totalForce.forces(:,rangeCut); 
shoes.Left.upsampled.totalForce.moments = shoes.Left.upsampled.totalForce.moments(:,rangeCut); 
% RIGHT----------------------------------------------------------------
shoes.Right.upsampled.time = shoes.Left.upsampled.time;
%
shoes.Right.upsampled.frontForce.forces  = shoes.Right.upsampled.frontForce.forces(:,rangeCut); 
shoes.Right.upsampled.frontForce.moments = shoes.Right.upsampled.frontForce.moments(:,rangeCut);
%
shoes.Right.upsampled.rearForce.forces  = shoes.Right.upsampled.rearForce.forces(:,rangeCut); 
shoes.Right.upsampled.rearForce.moments = shoes.Right.upsampled.rearForce.moments(:,rangeCut);
%
shoes.Right.upsampled.totalForce.forces  = shoes.Right.upsampled.totalForce.forces(:,rangeCut); 
shoes.Right.upsampled.totalForce.moments = shoes.Right.upsampled.totalForce.moments(:,rangeCut);



% % % Remove from signal NaN elements.  Two important assumptions are made:
% % % 1. NaN appears in the signal in the following way:
% % %    NaN NaN NaN ... value value value value ... NaN NaN NaN
% % % 2. the column of the indexInf (the last before the values) and the column
% % %    of the indexSup (the first after the values) are the same for both
% % %    Left (front/rear/left) and Right (front/rear/left).
% % 
% % lenUp = size(shoes.Left.upsampled.time,2);
% % for i = 1 : lenUp 
% %     [~, col] = find(isnan(shoes.Left.upsampled.totalForce.forces));
% % end
% % col_diff = diff(col);
% % for j = 1 : length(col)-1
% %     if col_diff(j) > 1
% %         indexInf = col(j);   % NaN from the beginning to indexInf
% %         indexSup = col(j+1); % NaN from indexSup to the end
% %     end
% % end

%% Extract subject weight
% The weight of the subject is the sum of the total forces (the computation
% omits NaN terms if there is any!).

bucket.weight = (abs(mean(shoes.Right.upsampled.totalForce.forces(3,:),'omitnan')) ...
                + abs(mean(shoes.Left.upsampled.totalForce.forces(3,:),'omitnan')))/9.81;

%% Clear useless variables
clearvars admissibleTreshold vecDim;