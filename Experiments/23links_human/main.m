
%----------------------------------------------------------------------
% UW_experiments:
% ---------------
% Xsens frequency  --> 240 Hz (see in suit.properties.frameRate)
% shoes frequency  --> 100 Hz (see from data)
% cortex frequency --> 100 Hz (both for MoCap + forcelates)
%----------------------------------------------------------------------

clear;clc;close all;
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
subjectID = 2;
trialID = 2;
bucket.pathToSubject = sprintf(fullfile(pwd,'/dataUW/Subj_%02d'),subjectID);
bucket.pathToTrial   = sprintf(fullfile(bucket.pathToSubject,'/trial_0%02d'),trialID);

%% Load measurements from SUIT
if ~exist(fullfile(bucket.pathToTrial,'suit.mat'))
    bucket.mvnxFilename = sprintf(fullfile(bucket.pathToTrial,'Subject_%02d-0%02d.mvnx'), subjectID, trialID);
    suit = extractSuitData(bucket.mvnxFilename);
    suit = computeSuitSensorPosition(suit); % obtain sensors position
    save(fullfile(bucket.pathToTrial,'/suit.mat'),'suit');
else
    load(fullfile(bucket.pathToTrial,'suit.mat'));
end

%% Define force data modality (shoes or forceplates)
shoes_bool       = true;     % if true --> shoes + Xsens
forceplates_bool = false;      % if true --> forceplates + Xsens

%% Choose and synchronize FORCE measurements combination
%SHOES
if shoes_bool 
     synchro_suit_and_shoes
end

%FORCE PLATES 
if forceplates_bool 
    synchro_suit_and_forceplates
end

%% Extract subject parameters from SUIT
subjectParamsFromData = subjectParamsComputation(suit, bucket.weight);

%% Transform forces into human forces
% Preliminary assumption on contact links: 2 contacts only (or both feet 
% with the shoes or both feet with two force plates)
bucket.contactLink = cell(2,1); 

%SHOES
if shoes_bool 
    % Define contacts configuration
    bucket.contactLink{1} = 'RightFoot'; % human link in contact with ftShoe_Right
    bucket.contactLink{2} = 'LeftFoot';  % human link in contact with ftShoe_Left
    shoes = transformShoesWrenches(shoes, subjectParamsFromData);
end


%FORCE PLATES 
if forceplates_bool
    % Define contacts configuration for UW setup
    bucket.contactLink{1} = 'RightFoot'; % human link in contact with forceplate 2
    bucket.contactLink{2} = 'LeftFoot';  % human link in contact with forceplate 1
    forceplates = transformForceplatesWrenches (forceplates, subjectParamsFromData);
end 

%% Create URDF model
bucket.filenameURDF = sprintf(fullfile(bucket.pathToSubject,'XSensURDF_subj%02d_48dof.urdf'), subjectID);
if ~exist(sprintf(fullfile(bucket.pathToSubject,'XSensURDF_subj%02d_48dof.urdf'), subjectID))
    bucket.URDFmodel = createXsensLikeURDFmodel(subjectParamsFromData, ...
                                                suit.sensors,...
                                                'filename',bucket.filenameURDF,...
                                                'GazeboModel',false);
end                                 
                                        
%% Create OSIM model
bucket.filenameOSIM = sprintf(fullfile(bucket.pathToSubject,'XSensOSIM_subj%02d_48dof.osim'), subjectID);
if ~exist(sprintf(fullfile(bucket.pathToSubject,'XSensOSIM_subj%02d_48dof.osim'), subjectID))
    bucket.OSIMmodel = createXsensLikeOSIMmodel(subjectParamsFromData, ...
                                                bucket.filenameOSIM);
end   

%% Inverse Kinematic computation 

if ~exist(fullfile(bucket.pathToTrial,'human_state.mat'))
    bucket.setupFile = fullfile(pwd,'/dataUW/fileSetup.xml');
    bucket.trcFile = sprintf('dataUW/Subj_%02d/opensim-0%02d.trc',subjectID, trialID);
    [human_state, human_ddq, selectedJoints] = IK(bucket.filenameOSIM, ...
                                                bucket.trcFile, ...
                                                bucket.setupFile,...
                                               rangeCut);
    % here selectedJoints is the order of the Osim computation.
    save(fullfile(bucket.pathToTrial,'/human_state.mat'),'human_state');
    save(fullfile(bucket.pathToTrial,'/human_ddq.mat'),'human_ddq');
    save(fullfile(bucket.pathToTrial,'/selectedJoints.mat'),'selectedJoints');
else
    load(fullfile(bucket.pathToTrial,'/human_state.mat'),'human_state');
    load(fullfile(bucket.pathToTrial,'/human_ddq.mat'),'human_ddq');
    load(fullfile(bucket.pathToTrial,'/selectedJoints.mat'),'selectedJoints');
end

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
if shoes_bool 
  fext.rightHuman = shoes.Right.upsampled.totalForce.humanFootWrench;
  fext.leftHuman  = shoes.Left.upsampled.totalForce.humanFootWrench;
end

if forceplates_bool 
 %to be completed
end

data = dataPackaging(humanModel,... 
                     humanSensors,...
                     suit,...
                     fext,...
                     human_ddq,...
                     bucket.contactLink);
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
% (excluding accelerometers and gyroscope for which already exists the
% iDynTree option).
sensorsToBeRemoved = [];
temp = struct;
temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
temp.id = 'LeftHand';
sensorsToBeRemoved = [sensorsToBeRemoved; temp];
temp = struct;
temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
temp.id = 'RightHand';
sensorsToBeRemoved = [sensorsToBeRemoved; temp];

if ~exist(fullfile(bucket.pathToTrial,'mu_dgiveny.mat'))
    % [mu_dgiveny_3sens, Sigma_specific_3sens] = MAPcomputation(berdy, human_state, y, priors, 'SENSORS_TO_REMOVE', sensorsToBeRemoved);
    % [mu_dgiveny_ALLsens, Sigma_dgiveny_ALLsens] = MAPcomputation(berdy, human_state, y, priors);
    [mu_dgiveny, ~] = MAPcomputation(berdy, human_state, y, priors, 'SENSORS_TO_REMOVE', sensorsToBeRemoved);
    save(fullfile(bucket.pathToTrial,'/mu_dgiveny.mat'),'mu_dgiveny');
    %  save(fullfile(bucket.pathToTrial,'/Sigma_dgiveny.mat'),'Sigma_dgiveny');
    else
    load(fullfile(bucket.pathToTrial,'/mu_dgiveny.mat'),'mu_dgiveny');
end


