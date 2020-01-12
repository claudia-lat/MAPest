

% -----------------------------------------------------------------------%
%  EXTERNAL FORCES
% -----------------------------------------------------------------------%
y_sim_fext = struct;
y_sim_fext.order = dVectorOrder;
y_sim_fext.meas = cell(length(dVectorOrder),1);

for vectOrderIdx = 1 : length(dVectorOrder)
    range_fextMEAS = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, dVectorOrder{vectOrderIdx}, opts.stackOfTaskMAP);
    y_sim_fext.meas{vectOrderIdx,1} = y_sim((range_fextMEAS:range_fextMEAS+5),:);
end
save(fullfile(bucket.pathToProcessedData,'y_sim_fext.mat'),'y_sim_fext');

if ~opts.stackOfTaskMAP
    % -----------------------------------------------------------------------%
    %  LIN ACCELERATION
    % -----------------------------------------------------------------------%
    y_sim_linAcc = struct;
    nrOfLinAccelerometer = 17;
    y_sim_linAcc.order = cell(nrOfLinAccelerometer,1);
    y_sim_linAcc.meas = cell(nrOfLinAccelerometer,1);
    
    for accSensIdx = 1 : nrOfLinAccelerometer
        y_sim_linAcc.order{accSensIdx,1} = data(accSensIdx).id;
        
        range_linAccMEAS = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, data(accSensIdx).id, opts.stackOfTaskMAP);
        y_sim_linAcc.meas{accSensIdx,1} = y_sim((range_linAccMEAS:range_linAccMEAS+2),:);
    end
    save(fullfile(bucket.pathToProcessedData,'y_sim_linAcc.mat'),'y_sim_linAcc');
    
    % -----------------------------------------------------------------------%
    %  JOINT ACCELERATION
    % -----------------------------------------------------------------------%
    y_sim_ddq = struct;
    % nrOfDdq = nrDofs;
    y_sim_ddq.order = selectedJoints;
    y_sim_ddq.meas = cell(nrDofs,1);
    
    for ddqIdx = 1 : nrDofs
        range_ddqMEAS = rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, selectedJoints{ddqIdx}, opts.stackOfTaskMAP);
        y_sim_ddq.meas{ddqIdx,1} = y_sim(range_ddqMEAS,:);
    end
    save(fullfile(bucket.pathToProcessedData,'y_sim_ddq.mat'),'y_sim_ddq');
end

