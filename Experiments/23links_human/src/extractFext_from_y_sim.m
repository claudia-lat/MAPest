
% -----------------------------------------------------------------------%
%  EXTERNAL FORCES
% -----------------------------------------------------------------------%
% Extraction of the following variables
% - RightFoot
% - LeftFoot
% - RightHand
% - LeftHand
% Note that the other applied (external) forces are null!


% ---RightFoot
range_fextMEAS_rightFoot = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'RightFoot');
y_sim(blockIdx).FextSim_RightFoot = y_sim(blockIdx).y_sim((range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5),:);

% ---LeftFoot
range_fextMEAS_leftFoot = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'LeftFoot');
y_sim(blockIdx).FextSim_LeftFoot = y_sim(blockIdx).y_sim((range_fextMEAS_leftFoot:range_fextMEAS_leftFoot+5),:);

% ---RightHand
range_fextMEAS_rightHand = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'RightHand');
y_sim(blockIdx).FextSim_RightHand = y_sim(blockIdx).y_sim((range_fextMEAS_rightHand:range_fextMEAS_rightHand+5),:);

% ---LeftHand
range_fextMEAS_leftHand = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'LeftHand');
y_sim(blockIdx).FextSim_LeftHand = y_sim(blockIdx).y_sim((range_fextMEAS_leftHand:range_fextMEAS_leftHand+5),:);
