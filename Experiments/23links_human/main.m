
%--------------------------------------------------------------------------
% JSI_experiments main
%--------------------------------------------------------------------------
clc;close all;clear all;
%clear;clc;close all;
rng(1); % forcing the casual generator to be const
format long;

%% Add src to the path
addpath(genpath('src'));
addpath(genpath('templates'));
addpath(genpath('../../src'));
addpath(genpath('../../external'));

%% Java path needed by OSIM
setupJAVAPath();

%% Preliminaries
% Create a structure 'bucket' where storing different stuff generating by
% running the code
bucket = struct;
subjectID = 1;
taskID = 0;

bucket.datasetRoot = fullfile(pwd, 'dataJSI');
%bucket.datasetRoot = fullfile('D:\Datasets\2018_Feb_JSI');

bucket.pathToSubject = fullfile(bucket.datasetRoot, sprintf('S%02d',subjectID));
bucket.pathToTask    = fullfile(bucket.pathToSubject,sprintf('task%d',taskID));
bucket.pathToRawData = fullfile(bucket.pathToTask,'data');
bucket.pathToProcessedData   = fullfile(bucket.pathToTask,'processed');

% Extraction of the masterFile
masterFile = load(fullfile(bucket.pathToRawData,sprintf(('S%02d_%02d.mat'),subjectID,taskID)));

% Option for computing the estimated Sigma
opts.Sigma_dgiveny = false;

%% ---------------------UNA TANTUM PROCEDURE-------------------------------
%% SUIT struct creation
if ~exist(fullfile(bucket.pathToProcessedData,'suit.mat'), 'file')
    % 1) extract data from C++ parsed files
    extractSuitDataFromParsing;
    % 2) compute sensor position
    suit = computeSuitSensorPosition(suit);
    save(fullfile(bucket.pathToProcessedData,'suit.mat'),'suit');
else
    load(fullfile(bucket.pathToProcessedData,'suit.mat'));
end

%% IMPORTANT NOTE:
% The subjects performed the experimental tasks with the drill on the right
% hand. This code will be modified for taking into account the presence of
% the drill. URDF/OSIM models and IK computation will be affected
% from this change.

%% Extract subject parameters from SUIT
subjectParamsFromData = subjectParamsComputation(suit, masterFile.Subject.Info.Weight);

%% Create URDF model
bucket.filenameURDF = fullfile(bucket.pathToSubject, sprintf('XSensURDF_subj%02d_48dof.urdf', subjectID));
if ~exist(bucket.filenameURDF, 'file')
    bucket.URDFmodel = createXsensLikeURDFmodel(subjectParamsFromData, ...
        suit.sensors,...
        'filename',bucket.filenameURDF,...
        'GazeboModel',false);
end

%% Create OSIM model
bucket.filenameOSIM = fullfile(bucket.pathToSubject, sprintf('XSensOSIM_subj%02d_48dof.osim', subjectID));
if ~exist(bucket.filenameOSIM, 'file')
    bucket.OSIMmodel = createXsensLikeOSIMmodel(subjectParamsFromData, ...
        bucket.filenameOSIM);
end

%% Inverse Kinematic computation
if ~exist(fullfile(bucket.pathToProcessedData,'human_state_tmp.mat'), 'file')
    bucket.setupFile = fullfile(pwd, 'templates', 'setupOpenSimIKTool_Template.xml');
    bucket.trcFile   = fullfile(bucket.pathToRawData,sprintf('S%02d_%02d.trc',subjectID,taskID));
    bucket.motFile   = fullfile(bucket.pathToProcessedData,sprintf('S%02d_%02d.mot',subjectID,taskID));
    [human_state_tmp, human_ddq_tmp, selectedJoints, groundBasePose] = IK(bucket.filenameOSIM, ...
        bucket.trcFile, ...
        bucket.setupFile, ...
        suit.properties.frameRate, ...
        bucket.motFile);
    % here selectedJoints is the order of the Osim computation.
    save(fullfile(bucket.pathToProcessedData,'human_state_tmp.mat'),'human_state_tmp');
    save(fullfile(bucket.pathToProcessedData,'human_ddq_tmp.mat'),'human_ddq_tmp');
    save(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'),'selectedJoints');
    save(fullfile(bucket.pathToProcessedData,'groundBasePose.mat'),'groundBasePose');
else
    load(fullfile(bucket.pathToProcessedData,'human_state_tmp.mat'));
    load(fullfile(bucket.pathToProcessedData,'human_ddq_tmp.mat'));
    load(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'));
    load(fullfile(bucket.pathToProcessedData,'groundBasePose.mat'));
end

%% Raw data handling
rawDataHandling;

%% Transform forces into human forces
% Preliminary assumption on contact links: 2 contacts only (or both feet
% with the shoes or both feet with two force plates)
bucket.contactLink = cell(2,1);

% Define contacts configuration
bucket.contactLink{1} = 'RightFoot'; % human link in contact with RightShoe
bucket.contactLink{2} = 'LeftFoot';  % human link in contact with LeftShoe
for blockIdx = 1 : block.nrOfBlocks
    shoes(blockIdx) = transformShoesWrenches(synchroData(blockIdx), subjectParamsFromData);
end

%% ------------------------RUNTIME PROCEDURE-------------------------------
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
humanSensors.removeAllSensorsOfType(iDynTree.GYROSCOPE_SENSOR);
bucket.base = 'LeftFoot'; % floating base

%% Initialize berdy
% Specify berdy options
berdyOptions = iDynTree.BerdyOptions;

berdyOptions.baseLink = bucket.base;
berdyOptions.includeAllNetExternalWrenchesAsSensors          = true;
berdyOptions.includeAllNetExternalWrenchesAsDynamicVariables = true;
berdyOptions.includeAllJointAccelerationsAsSensors           = true;
berdyOptions.includeAllJointTorquesAsSensors                 = false;

berdyOptions.berdyVariant = iDynTree.BERDY_FLOATING_BASE;
berdyOptions.includeFixedBaseExternalWrench = false;

% Load berdy
berdy = iDynTree.BerdyHelper;
berdy.init(humanModel, humanSensors, berdyOptions);
% Get the current traversal
traversal = berdy.dynamicTraversal;
currentBase = berdy.model().getLinkName(traversal.getBaseLink().getIndex());
disp(strcat('Current base is < ', currentBase,'>.'));
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

% ---------------------------------------------------
% CHECK: print the order of variables in d vector
% printBerdyDynVariables_floating(berdy)
% ---------------------------------------------------

%% Measurements wrapping
for blockIdx = 1 : block.nrOfBlocks
    fext.rightHuman = shoes(blockIdx).Right_HF;
    fext.leftHuman  = shoes(blockIdx).Left_HF;
    
    data(blockIdx).block = block.labels(blockIdx);
    data(blockIdx).data  = dataPackaging(blockIdx, ...
        humanModel,...
        humanSensors,...
        suit_runtime,...
        fext,...
        synchroData(blockIdx).ddq,...
        bucket.contactLink);
    % y vector as input for MAP
    [data(blockIdx).y, data(blockIdx).Sigmay] = berdyMeasurementsWrapping(berdy, ...
        data(blockIdx).data);
end

% ---------------------------------------------------
% CHECK: print the order of measurement in y
% printBerdySensorOrder(berdy);
% ---------------------------------------------------

%% ------------------------------- MAP ------------------------------------
%% Set priors
priors        = struct;
priors.mud    = zeros(berdy.getNrOfDynamicVariables(), 1);
priors.Sigmad = 1e+4 * eye(berdy.getNrOfDynamicVariables());
priors.SigmaD = 1e-4 * eye(berdy.getNrOfDynamicEquations());

%% Possibility to remove a sensor from the analysis
% excluding the accelerometers and gyroscope for whose removal already
% exists the iDynTree option.
sensorsToBeRemoved = [];

bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
bucket.temp.id = 'LeftHand';
sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];
% %
bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
bucket.temp.id = 'RightHand';
sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];
% %
% % bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% % bucket.temp.id = 'RightFoot';
% % sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];
% %
% % bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% % bucket.temp.id = 'LeftFoot';
% % sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];

%% Angular velocity
% related to the chosen current base and to use it into the MAP
% computation.
bucket.baseAngVel = [0 0 0]; % forced to be zero, acceptable hp for this task.
% NOTE: we do not have this info anymore from sensors!

% for i = 1 : length(suit_downsampled.sensors)
%     if strcmp(suit_downsampled.sensors{i, 1}.label, currentBase)
%         bucket.baseAngVel = suit_downsampled.sensors{i, 1}.meas.sensorAngularVelocity;
%         break;
%     end
% end

%% MAP computation
if ~exist(fullfile(bucket.pathToProcessedData,'estimation.mat'), 'file')
    for blockIdx = 1 : block.nrOfBlocks
        priors.Sigmay = data(blockIdx).Sigmay;
        estimation(blockIdx).block = block.labels(blockIdx);
        if opts.Sigma_dgiveny
            [estimation(blockIdx).mu_dgiveny, estimation(blockIdx).Sigma_dgiveny] = MAPcomputation_floating(berdy, ...
                traversal, ...
                synchroData(blockIdx), ...
                data(blockIdx).y, ...
                priors, ...
                bucket.baseAngVel, ...
                'SENSORS_TO_REMOVE', sensorsToBeRemoved);
            % TODO: variables extraction
            % Sigma_tau extraction from Sigma d --> since sigma d is very big, it
            % cannot be saved! therefore once computed it is necessary to extract data
            % related to tau and save that one!
            % TODO: extractSigmaOfEstimatedVariables
        else
            [estimation(blockIdx).mu_dgiveny] = MAPcomputation_floating(berdy, ...
                traversal, ...
                synchroData(blockIdx), ...
                data(blockIdx).y, ...
                priors, ...
                bucket.baseAngVel, ...
                'SENSORS_TO_REMOVE', sensorsToBeRemoved);
        end
    end
    save(fullfile(bucket.pathToProcessedData,'estimation.mat'),'estimation');
else
    load(fullfile(bucket.pathToProcessedData,'estimation.mat'));
end

%% Variables extraction
% torques
if ~exist(fullfile(bucket.pathToProcessedData,'computedTauFromBerdy.mat'), 'file')
    extractTauFromBerdy
    save(fullfile(bucket.pathToProcessedData,'computedTauFromBerdy.mat'),'computedTauFromBerdy');
else
    load(fullfile(bucket.pathToProcessedData,'computedTauFromBerdy.mat'));
end
% external forces
if ~exist(fullfile(bucket.pathToProcessedData,'computedFext.mat'), 'file')
    extractFext
    save(fullfile(bucket.pathToProcessedData,'computedFext.mat'),'computedFext');
else
    load(fullfile(bucket.pathToProcessedData,'computedFext.mat'));
end
