
clear;clc;close all;
rng(1); % forcing the casual generator to be const

%% Add src to the path
addpath(genpath('src')); 
addpath(genpath('templates')); 
addpath(genpath('../../src'));

% create a structure 'bucket' where storing different stuff generating by
% running the code
bucket = struct;
%% Java path needed by OSIM
bucket.JAVA_OSIM_PATH = '/usr/local/Cellar/opensim-core/4.0/share/doc/OpenSim/Java';
bucket.JAVA_OSIM_LIB_FOLDER = '/usr/local/Cellar/opensim-core/4.0/lib';
%get current class path
bucket.javacpath = javaclasspath('-dynamic');
found = false;
for i = 1:size(bucket.javacpath)
    if (strcmp(bucket.javacpath{i}, bucket.JAVA_OSIM_PATH))
        found = true;
        break;
    end
end
if ~found
    javaaddpath(bucket.JAVA_OSIM_PATH);
end
bucket.javapathFile = fullfile(prefdir, 'javalibrarypath.txt');
found = false;
if exist(bucket.javapathFile, 'file')
    %open and check file
    bucket.fid = fopen(bucket.javapathFile,'r');
    %for each line, trim and compare with JAVA_OSIM_LIB_FOLDER
    bucket.tline = fgetl(bucket.fid);
    while ischar(bucket.tline)
        if strcmp(strtrim(bucket.tline), bucket.JAVA_OSIM_LIB_FOLDER)
            found = true;
            break;
        end
        bucket.tline = fgetl(bucket.fid);
    end
    fclose(bucket.fid);
end
if ~found
    bucket.fid = fopen(bucket.javapathFile,'a');
    fprintf(bucket.fid, strcat(bucket.JAVA_OSIM_LIB_FOLDER, '\n'));
    fclose(bucket.fid);
end

%% Load measurements from SUIT
bucket.mvnxFilename = 'data/Meri-002.mvnx';
% suit = extractSuitData(bucket.mvnxFilename,'data');
% suit = computeSuitSensorPosition(suit); % obtain sensors position

%% Load measurements from FORCEPLATES and ROBOT
bucket.AMTIfilename          = 'data/AMTIdata002.txt';
bucket.TSfilename            = 'data/TSdata002.txt';
bucket.ROBOTfilenameRight    = 'data/robotRight002.txt';
bucket.ROBOTfilenameLeft     = 'data/robotLeft002.txt';
bucket.rightArmStateFilename = 'data/rightArmState.txt';
bucket.leftArmStateFilename  = 'data/leftArmState.txt';
bucket.rightLegStateFilename = 'data/rightLegState.txt';
bucket.leftLegStateFilename  = 'data/leftLegState.txt';
bucket.torsoStateFilename    = 'data/torsoState.txt';
% -------------------------------------------------------------------------
% Following configuration is customized for this particular experiment:%
bucket.contactLink = cell(4,1);
bucket.contactLink{1} = 'RightFoot'; % human link in contact with forceplate 1
bucket.contactLink{2} = 'LeftFoot';  % human link in contact with forceplate 2
bucket.contactLink{3} = 'LeftHand';  % human link in contact with right robot forearm
bucket.contactLink{4} = 'RightHand'; % human link in contact with left robot forearm
% -------------------------------------------------------------------------
%% Extract and synchronised measurements
bucket.suitTimeInit = suit.time;
[forceplate, bucket.suitIndex] = extractForceplateData(bucket.AMTIfilename, ...
                                                       bucket.TSfilename, ...
                                                       bucket.suitTimeInit, ...
                                                       bucket.contactLink,...
                                                       'outputdir', 'data');
bucket.timeSeries = bucket.suitTimeInit(bucket.suitIndex);

[robot, bucket.syncIndex] = extractRobotData(bucket.ROBOTfilenameLeft, ...
                                             bucket.ROBOTfilenameRight, ...
                                             bucket.timeSeries, ...
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
                                             bucket.suitTimeInit);

%% Extract subject parameters from SUIT
subjectID = 1;
bucket.M = 60;
subjectParamsFromData = subjectParamsComputation(suit, bucket.M);

%% Process raw data from forceplates
forceplate = processForceplateWrenches(forceplate, subjectParamsFromData);

%% Create URDF model
bucket.filenameURDF = sprintf('models/XSensURDF_subj%d.urdf',subjectID);
bucket.URDFmodel = createXsensLikeURDFmodel(subjectParamsFromData, ...
                                            suit.sensors,...
                                            'filename',bucket.filenameURDF,...
                                            'GazeboModel',true);

%% Create OSIM model
bucket.filenameOSIM = sprintf('models/XSensOSIM_subj%d.osim',subjectID);
bucket.OSIMmodel = createXsensLikeOSIMmodel(subjectParamsFromData, ...
                                            bucket.filenameOSIM);

%% Inverse Kinematic computation 
bucket.setupFile = ('data/fileSetup.xml');
bucket.trcFile = ('data/Meri-002.trc');
[human_state, human_ddq, selectedJoints] = IK(bucket.filenameOSIM, ...
                                            bucket.trcFile, ...
                                            bucket.setupFile,...
                                            suitSyncIndex);
% here selectedJoints is the order of the Osim computation.
%% Load URDF model with sensors
humanModel.filename = bucket.filenameURDF;
humanModelLoader = iDynTree.ModelLoader();
if ~humanModelLoader.loadReducedModelFromFile(humanModel.filename, cell2iDynTreeStringVector(selectedJoints));
    % here the model loads the same order.
    fprintf('Something wrong with the model loading.')
end
humanModel = humanModelLoader.model();
human_kinDynComp = iDynTree.KinDynComputations();
human_kinDynComp.loadRobotModel(humanModel);

humanSensors = humanModelLoader.sensors();
% remove left foot sensor since it will be the fixed base
fixedBase = 'LeftFoot';
humanSensors.removeSensor(iDynTree.ACCELEROMETER_SENSOR, strcat(fixedBase,'_accelerometer'));
humanSensors.removeSensor(iDynTree.GYROSCOPE_SENSOR, strcat(fixedBase,'_gyro'));
% humanSensors.removeSensor(iDynTree.ACCELEROMETER_SENSOR, 'LeftFoot_accelerometer');
% humanSensors.removeSensor(iDynTree.GYROSCOPE_SENSOR, 'LeftFoot_gyro');
% we decided to remove gyroscopes from the analysis
humanSensors.removeAllSensorsOfType(iDynTree.GYROSCOPE_SENSOR);

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

%% initialize berdy
% model   = humanModelLoader.model();
% sensors = humanModelLoader.sensors();
% specify berdy options
berdyOptions = iDynTree.BerdyOptions;
berdyOptions.baseLink = fixedBase; % change of the base link
% berdyOptions.baseLink = 'LeftFoot'; % change of the base link
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

% load berdy
berdy = iDynTree.BerdyHelper;
berdy.init(humanModel, humanSensors, berdyOptions);
% get the current traversal
traversal = berdy.dynamicTraversal;
currentBase = berdy.model().getLinkName(traversal.getBaseLink().getIndex());
disp(strcat('Current base is < ', currentBase,'>.'));
disp(strcat('Be sure that sensors in URDF related to <', currentBase,'> has been removed!'));
% get the tree is visited IS the ordere of variables in vector d

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

printBerdyDynVariables(berdy)

%% Measurements wrapping
data = dataPackaging(humanModel,... 
                     humanSensors,...
                     suit,...
                     forceplate,...
                     human_ddq,...
                     robot);
[y, Sigmay] = berdyMeasurementsWrapping(berdy, data);

%% MAP
% set priors
priors        = struct;
priors.mud    = zeros(berdy.getNrOfDynamicVariables(), 1);
priors.Sigmad = 1e+4 * eye(berdy.getNrOfDynamicVariables());
priors.SigmaD = 1e-4 * eye(berdy.getNrOfDynamicEquations());
priors.Sigmay = Sigmay;
tic;
[mu_dgiveny, Sigma_dgiveny] = MAPcomputation(berdy, human_state, y, priors);
% [mu_dgiveny, Sigma_dgiveny, modelError, measError] = MAPcomputation(berdy, human_state, y, priors);
toc;

 plotScript