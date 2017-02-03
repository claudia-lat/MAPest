
clear;clc;close all;
rng(1); % forcing the casual generator to be const

%% Add src to the path
addpath(genpath('src')); 
addpath(genpath('templates')); 
addpath(genpath('../../src'));

% Create a structure 'bucket' where storing different stuff generating by
% running the code
bucket = struct;
% Set trial number
trialID0 = [];
trialID = 20;

%% Java path needed by OSIM
setupJAVAPath();

%% Load measurements from SUIT
bucket.mvnxFilename = sprintf('data/Meri-0%d%d.mvnx',trialID0, trialID);
suit = extractSuitData(bucket.mvnxFilename,'data002');
suit = computeSuitSensorPosition(suit); % obtain sensors position

%% Load measurements from FORCEPLATES and ROBOT
bucket.AMTIfilename          = sprintf('data/AMTIdata0%d%d.txt',trialID0, trialID);
bucket.TSfilename            = sprintf('data/TSdata0%d%d.txt',trialID0, trialID);
bucket.ROBOTfilenameRight    = sprintf('data/robotRight0%d%d.txt',trialID0, trialID);
bucket.ROBOTfilenameLeft     = sprintf('data/robotLeft0%d%d.txt',trialID0, trialID);
bucket.rightArmStateFilename = sprintf('data/rightArmState0%d%d.txt',trialID0, trialID);
bucket.leftArmStateFilename  = sprintf('data/leftArmState0%d%d.txt',trialID0, trialID);
bucket.rightLegStateFilename = sprintf('data/rightLegState0%d%d.txt',trialID0, trialID);
bucket.leftLegStateFilename  = sprintf('data/leftLegState0%d%d.txt',trialID0, trialID);
bucket.torsoStateFilename    = sprintf('data/torsoState0%d%d.txt',trialID0, trialID);
% -------------------------------------------------------------------------
% Following configuration is customized for this particular experiment:%
bucket.contactLink = cell(4,1);
bucket.contactLink{1} = 'RightFoot'; % human link in contact with forceplate 1
bucket.contactLink{2} = 'LeftFoot';  % human link in contact with forceplate 2
bucket.contactLink{3} = 'LeftHand';  % human link in contact with right robot forearm
bucket.contactLink{4} = 'RightHand'; % human link in contact with left robot forearm
% -------------------------------------------------------------------------
%% Extract and synchronised measurements
[forceplate, bucket.suitIndex] = extractForceplateData(bucket.AMTIfilename, ...
                                                       bucket.TSfilename, ...
                                                       bucket.contactLink,...
                                                       'outputdir', 'data');

[robot, bucket.syncIndex] = extractRobotData(bucket.ROBOTfilenameLeft, ...
                                             bucket.ROBOTfilenameRight, ...
                                             forceplate.data.time.unixTime , ...
                                             bucket.contactLink,...
                                             bucket.rightArmStateFilename, ...
                                             bucket.leftArmStateFilename, ...
                                             bucket.rightLegStateFilename, ...
                                             bucket.leftLegStateFilename, ...
                                             bucket.torsoStateFilename,...
                                             'outputdir', 'data');                         
[suit, forceplate, suitSyncIndex] = dataSync(suit,...
                                             forceplate, ...
                                             bucket.syncIndex,...
                                             bucket.suitIndex);

%% Extract subject parameters from SUIT
subjectID = 1;
weight = (forceplate.data.plateforms.plateform1.forces(3,1) ...
          + forceplate.data.plateforms.plateform2.forces(3,1))/9.81;
bucket.M = weight;
subjectParamsFromData = subjectParamsComputation(suit, bucket.M);

%% Process raw data from forceplates
forceplate = processForceplateWrenches(forceplate, subjectParamsFromData);

%% Create URDF model
bucket.filenameURDF = sprintf('models/XSensURDF_subj%d.urdf',subjectID);
bucket.URDFmodel = createXsensLikeURDFmodel(subjectParamsFromData, ...
                                            suit.sensors,...
                                            'filename',bucket.filenameURDF,...
                                            'GazeboModel',false);

%% Create OSIM model
bucket.filenameOSIM = sprintf('models/XSensOSIM_subj%d.osim',subjectID);
bucket.OSIMmodel = createXsensLikeOSIMmodel(subjectParamsFromData, ...
                                            bucket.filenameOSIM);

%% Inverse Kinematic computation 
bucket.setupFile = ('data/fileSetup.xml');
bucket.trcFile = sprintf('data/Meri-0%d%d.trc',trialID0, trialID);
[human_state, human_ddq, selectedJoints] = IK(bucket.filenameOSIM, ...
                                            bucket.trcFile, ...
                                            bucket.setupFile,...
                                            suitSyncIndex);
% here selectedJoints is the order of the Osim computation.
%% Load URDF model with sensors
humanModel.filename = bucket.filenameURDF;
humanModelLoader = iDynTree.ModelLoader();
if ~humanModelLoader.loadReducedModelFromFile(humanModel.filename, ...
        cell2iDynTreeStringVector(selectedJoints))
% here the model loads the same order of selectedJoints.
fprintf('Something wrong with the model loading.')
end
humanModel = humanModelLoader.model();
human_kinDynComp = iDynTree.KinDynComputations();
human_kinDynComp.loadRobotModel(humanModel);

humanSensors = humanModelLoader.sensors();
% Remove sensor on the fixed base
fixedBase = 'LeftFoot';
humanSensors.removeSensor(iDynTree.ACCELEROMETER_SENSOR, strcat(fixedBase,'_accelerometer'));
humanSensors.removeSensor(iDynTree.GYROSCOPE_SENSOR, strcat(fixedBase,'_gyro'));
% We decided to remove gyroscopes from the analysis
humanSensors.removeAllSensorsOfType(iDynTree.GYROSCOPE_SENSOR);
% humanSensors.removeAllSensorsOfType(iDynTree.ACCELEROMETER_SENSOR);
%% Load robot URDF model
[robotJointPos, robotModel] = createRobotModel(robot);
robot_kinDynComp = iDynTree.KinDynComputations();
robot_kinDynComp.loadRobotModel(robotModel);

%% Process raw data from robot
for i= 1:robot.data.properties.nrOfFrame
robot = processRobotWrenches(robot_kinDynComp, ...
                             human_kinDynComp, ...
                             robotJointPos(:,i), ...
                             human_state.q(:,i), ...
                             robot, ...
                             subjectParamsFromData);
end

%% Initialize berdy
% Specify berdy options
berdyOptions = iDynTree.BerdyOptions;
berdyOptions.baseLink = fixedBase; % change of the base link
%--------------------------------------------------------------------------
% IMPORTANT NOTE:
% ---------------
% Until this point the base link was 'Pelvis' but from now on  we decided
% to change it with the 'LeftFoot' since it is really fixed during the
% experiment.
%--------------------------------------------------------------------------
berdyOptions.includeAllNetExternalWrenchesAsSensors          = true;
berdyOptions.includeAllNetExternalWrenchesAsDynamicVariables = true;
berdyOptions.includeAllJointAccelerationsAsSensors           = true;
berdyOptions.includeAllJointTorquesAsSensors                 = false;
berdyOptions.includeFixedBaseExternalWrench                  = true;

% Load berdy
berdy = iDynTree.BerdyHelper;
berdy.init(humanModel, humanSensors, berdyOptions);
% Get the current traversal
traversal = berdy.dynamicTraversal;
currentBase = berdy.model().getLinkName(traversal.getBaseLink().getIndex());
disp(strcat('Current base is < ', currentBase,'>.'));
disp(strcat('Be sure that sensors in URDF related to <', currentBase,'> has been removed!'));

% Get the tree is visited IS the order of variables in vector d
dVectorOrder = cell(traversal.getNrOfVisitedLinks(), 1); %for each link in the traversal get the name
dJointOrder = cell(traversal.getNrOfVisitedLinks()-1, 1);
for i = 0 : traversal.getNrOfVisitedLinks() - 1
    if i ~= 0 
        joint  = traversal.getParentJoint(i);
        dJointOrder{i} = berdy.model().getJointName(joint.getIndex());
    end
    link = traversal.getLink(i);
    dVectorOrder{i + 1} = berdy.model().getLinkName(link.getIndex());
end

% -------------------------------------------------------------------------
% CHECK: print the order of variables in d vector
%printBerdyDynVariables(berdy)
% -------------------------------------------------------------------------
%% Measurements wrapping
data = dataPackaging(humanModel,... 
                     humanSensors,...
                     suit,...
                     forceplate,...
                     human_ddq,...
                     robot);
[y, Sigmay] = berdyMeasurementsWrapping(berdy, data);
% -------------------------------------------------------------------------
% CHECK: print the order of measurement in y 
% printBerdySensorOrder(berdy);
% -------------------------------------------------------------------------
%% MAP
% Set priors
priors        = struct;
priors.mud    = zeros(berdy.getNrOfDynamicVariables(), 1);
priors.Sigmad = 1e+4 * eye(berdy.getNrOfDynamicVariables());
priors.SigmaD = 1e-4 * eye(berdy.getNrOfDynamicEquations());
priors.Sigmay = Sigmay;

% Added the possibility to remove a sensor from the analysis
% (excluding accelerometers and gyroscope for wich already exist the 
% iDynTree opton).
sensorsToBeRemoved = [];
temp = struct;
temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
temp.id = 'LeftHand';
sensorsToBeRemoved = [sensorsToBeRemoved; temp];
temp = struct;
temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
temp.id = 'RightHand';
sensorsToBeRemoved = [sensorsToBeRemoved; temp];

% [mu_dgiveny_3sens, Sigma_specific_3sens] = MAPcomputation(berdy, human_state, y, priors, 'SENSORS_TO_REMOVE', sensorsToBeRemoved);
[mu_dgiveny_ALLsens, Sigma_dgiveny_ALLsens] = MAPcomputation(berdy, human_state, y, priors);

% bowingPlot
% squatPlot
% addingSensorsPlot
