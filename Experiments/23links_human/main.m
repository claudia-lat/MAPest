
%----------------------------------------------------------------------
% UW_experiments:
% ---------------
% Xsens frequency  --> 240 Hz (see in suit.properties.frameRate)
% shoes frequency  --> 100 Hz (see from data)
% cortex frequency --> 100 Hz (both for MoCap + forcelates)
%----------------------------------------------------------------------

% clearvars -except shoes_old;
clc;close all;clear all;
%clear;clc;close all;
rng(1); % forcing the casual generator to be const
format long;

%% Add src to the path
addpath(genpath('src')); 
addpath(genpath('templates')); 
addpath(genpath('../../src'));
addpath(genpath('../../external'));
addpath(genpath('dataAnalysis_thesis'));

%% Java path needed by OSIM
setupJAVAPath();

%% Preliminaries
% Create a structure 'bucket' where storing different stuff generating by
% running the code
bucket = struct;
subjectID = 10;
trialID = 5;
bucket.pathToSubject = sprintf(fullfile(pwd,'/dataUW/Subj_%02d'),subjectID);
bucket.pathToTrial   = sprintf(fullfile(bucket.pathToSubject,'/trial_0%02d'),trialID);
bucket.pathToProcessedData   = fullfile(bucket.pathToTrial,'/processed');

%% Load measurements from SUIT
if ~exist(fullfile(bucket.pathToProcessedData,'suit.mat'))
    bucket.mvnxFilename = sprintf(fullfile(bucket.pathToTrial,...
                                  'Subject_%02d-0%02d.mvnx'), subjectID, trialID);
    suit = extractSuitData(bucket.mvnxFilename);
    suit = computeSuitSensorPosition(suit); % obtain sensors position
    save(fullfile(bucket.pathToProcessedData,'/suit.mat'),'suit');
else
    load(fullfile(bucket.pathToProcessedData,'suit.mat'));
end

%% Define force data modality (shoes or forceplates)
shoes_bool              = false;     % if true --> shoes + Xsens
forceplates_bool        = false;    % if true --> forceplates + Xsens
shoesVSforceplates_bool = true;    % if true --> shoes + forceplates for the comparison

%% Choose and synchronize FORCE measurements combination
if shoes_bool | shoesVSforceplates_bool
    bucket.inFolder = fullfile(bucket.pathToProcessedData,'/shoes');
    if(exist(bucket.inFolder,'dir')==0)
        mkdir(bucket.inFolder);
    end
   synchro_suit_and_shoes
end
 
if forceplates_bool | shoesVSforceplates_bool
   bucket.inFolder = fullfile(bucket.pathToProcessedData,'/forceplates');
   if(exist(bucket.inFolder,'dir')==0)
        mkdir(bucket.inFolder);
   end
   synchro_suit_and_forceplates
end

%% Extract subject parameters from SUIT
subjectParamsFromData = subjectParamsComputation(suit, bucket.weight);

%% Transform forces into human forces
% Preliminary assumption on contact links: 2 contacts only (or both feet 
% with the shoes or both feet with two force plates)
bucket.contactLink = cell(2,1); 

if shoes_bool | shoesVSforceplates_bool
    % Define contacts configuration
    bucket.contactLink{1} = 'RightFoot'; % human link in contact with ftShoe_Right
    bucket.contactLink{2} = 'LeftFoot';  % human link in contact with ftShoe_Left
    shoes = transformShoesWrenches(shoes, subjectParamsFromData);
end
 
if forceplates_bool | shoesVSforceplates_bool
    % Define contacts configuration for UW setup
    bucket.contactLink{1} = 'RightFoot'; % human link in contact with forceplate 2 (UW setup)
    bucket.contactLink{2} = 'LeftFoot';  % human link in contact with forceplate 1 (UW setup)

    % The position of the forceplates are contained in the file
    % 'unloaded_fp_4markers1.trc'.
    bucket.filenameTrc = fullfile(bucket.pathToTrial,'/forceplates/unloaded_fp_4markers1.trc');
    forceplates = transformForceplatesWrenches (forceplates, subjectParamsFromData, ...
                                                bucket.filenameTrc);
end                               

%% If we consider the shoes and forceplates VALIDATION
if shoesVSforceplates_bool
   validation_shoes_and_forceplates
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
if ~exist(fullfile(bucket.inFolder,'human_state_tmp.mat'))
    bucket.setupFile = fullfile(pwd,'/dataUW/fileSetup.xml');
    bucket.trcFile = sprintf('dataUW/Subj_%02d/trial_0%02d/opensim-0%02d.trc',subjectID, trialID, trialID);
    [human_state_tmp, human_ddq_tmp, selectedJoints] = IK(bucket.filenameOSIM, ...
                                                bucket.trcFile, ...
                                                bucket.setupFile,...
                                                rangeCut);
    % here selectedJoints is the order of the Osim computation.
    save(fullfile(bucket.inFolder,'/human_state_tmp.mat'),'human_state_tmp');
    save(fullfile(bucket.inFolder,'/human_ddq_tmp.mat'),'human_ddq_tmp');
    save(fullfile(bucket.pathToProcessedData,'/selectedJoints.mat'),'selectedJoints');
else
    load(fullfile(bucket.inFolder,'/human_state_tmp.mat'),'human_state_tmp');
    load(fullfile(bucket.inFolder,'/human_ddq_tmp.mat'),'human_ddq_tmp');
    load(fullfile(bucket.pathToProcessedData,'/selectedJoints.mat'),'selectedJoints');
end

%% Downsampling of the data
%--------------------------------------------------------------------------
% IMPORTANT NOTE:
% ---------------
% This step is due here since the data needed by the MAP function are too
% big for the Matlab computation! Each signal (that at this step is already
% synchronized and properly cutted) will be here downsampled from 240Hz 
% to 100Hz as follows:

newSlaveTime  = suit.time.* 1.e-3; %to ms;
if shoes_bool
    newMasterTime = shoes.Left.interp_synch.time;
end
if forceplates_bool
    newSlaveTime  = suit_time_abs; % because the forceplates time is only absolute!
    clearvars suit_time_abs;
    newMasterTime = forceplates.time;
end

% ----- Suit ----
% WARNING: downsampled only the needed variables! 
% In this case is useful only suit.sensors
for i = 1 : suit.properties.nrOfSensors
    suit_downsampled.sensors{i,1}.label = suit.sensors{i}.label;
    for j = 1 : 3
        suit_downsampled.sensors{i,1}.meas.sensorAcceleration(j,:) = interp1(newSlaveTime, ...
            suit.sensors{i}.meas.sensorAcceleration(j,:), newMasterTime);
        suit_downsampled.sensors{i,1}.meas.sensorAngularVelocity(j,:) = interp1(newSlaveTime, ...
            suit.sensors{i}.meas.sensorAngularVelocity(j,:), newMasterTime);
%     suit.sensors{i}.meas.sensorMagneticField = suit.sensors{i}.meas.sensorMagneticField; 
%     suit.sensors{i}.meas.sensorOrientation = suit.sensors{i}.meas.sensorOrientation; 
    end
end

% ----- Human_state and ddq ----
% WARNING: downsampled only the needed variables!
human_state.q  = zeros(size(human_ddq_tmp,1),size(newMasterTime,2));
human_state.dq = zeros(size(human_ddq_tmp,1),size(newMasterTime,2));
human_ddq      = zeros(size(human_ddq_tmp,1),size(newMasterTime,2));
for i = 1 : size(human_ddq,1)
    human_state.q(i,:)  = interp1(newSlaveTime, human_state_tmp.q(i,:), newMasterTime);
    human_state.dq(i,:) = interp1(newSlaveTime, human_state_tmp.dq(i,:), newMasterTime);
    human_ddq(i,:)      = interp1(newSlaveTime, human_ddq_tmp(i,:), newMasterTime);
end
clearvars human_state_tmp human_ddq_tmp;

% ----- Shoes OR forceplates ----
% WARNING: downsampled only the needed variables!
if shoes_bool  
    shoes.Left.downsampled.time  = newMasterTime;
    shoes.Right.downsampled.time = newMasterTime;
    % Let's consider only the forces already transformed in human frames.
    % If necessary, frontForce/rearForce/totalForce are exactly the interp_synch ones!
    for i = 1 : size(shoes.Left.upsampled.totalForce.humanFootWrench,1)
        % LEFT-------------------------------------------------------------
        shoes.Left.downsampled.totalForce.humanFootWrench(i,:) = interp1(newSlaveTime,...
            shoes.Left.upsampled.totalForce.humanFootWrench(i,:), newMasterTime); 
        % RIGHT------------------------------------------------------------
        shoes.Right.downsampled.totalForce.humanFootWrench(i,:) = interp1(newSlaveTime,...
            shoes.Right.upsampled.totalForce.humanFootWrench(i,:), newMasterTime); 
    end
    save(fullfile(bucket.inFolder,'/shoes.mat'),'shoes');
end

if forceplates_bool 
    forceplates.downsampled.time = newMasterTime;
    % Let's consider only the forces already transformed in human frames.
        for i = 1 : size(forceplates.upsampled.FP1.humanLeftFootWrench,1)
        % FP1--------------------------------------------------------------
        forceplates.downsampled.FP1.humanLeftFootWrench(i,:) = interp1(newSlaveTime,...
            forceplates.upsampled.FP1.humanLeftFootWrench(i,:), newMasterTime); 
        % FP2--------------------------------------------------------------
        forceplates.downsampled.FP2.humanRightFootWrench(i,:) = interp1(newSlaveTime,...
            forceplates.upsampled.FP2.humanRightFootWrench(i,:), newMasterTime); 
        end
    save(fullfile(bucket.inFolder,'/forceplates.mat'),'forceplates');
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
% printBerdyDynVariables(berdy)
% -------------------------------------------------------------------------
%% Measurements wrapping
if shoes_bool 
  fext.rightHuman = shoes.Right.downsampled.totalForce.humanFootWrench;
  fext.leftHuman  = shoes.Left.downsampled.totalForce.humanFootWrench;
end

if forceplates_bool 
  fext.rightHuman = forceplates.downsampled.FP2.humanRightFootWrench;
  fext.leftHuman  = forceplates.downsampled.FP1.humanLeftFootWrench;
end

data = dataPackaging(humanModel,... 
                     humanSensors,...
                     suit_downsampled,...
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

temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
temp.id = 'RightHand';
sensorsToBeRemoved = [sensorsToBeRemoved; temp];

% temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% temp.id = 'RightFoot';
% sensorsToBeRemoved = [sensorsToBeRemoved; temp];
%
% temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% temp.id = 'LeftFoot';
% sensorsToBeRemoved = [sensorsToBeRemoved; temp];

% Vector d computation
if ~exist(fullfile(bucket.inFolder,'mu_dgiveny.mat'))
    % [mu_dgiveny_3sens, Sigma_specific_3sens] = MAPcomputation(berdy, human_state, y, priors, 'SENSORS_TO_REMOVE', sensorsToBeRemoved);
    % [mu_dgiveny_ALLsens, Sigma_dgiveny_ALLsens] = MAPcomputation(berdy, human_state, y, priors);
    [mu_dgiveny, ~] = MAPcomputation(berdy, human_state, y, priors, 'SENSORS_TO_REMOVE', sensorsToBeRemoved);
    save(fullfile(bucket.inFolder,'/mu_dgiveny.mat'),'mu_dgiveny');
    else
    load(fullfile(bucket.inFolder,'/mu_dgiveny.mat'),'mu_dgiveny');
end

% Sigma_tau extraction from Sigma d --> since sigma d is very big, it
% cannot be saved! therefore once computed it is necessary to extract data
% related to tau and save that one!
if ~exist(fullfile(bucket.inFolder,'Sigma_tau.mat'))
    [~, Sigma_dgiveny] = MAPcomputation(berdy, human_state, y, priors, 'SENSORS_TO_REMOVE', sensorsToBeRemoved);
    extractSigmaOfEstimatedVariables
    save(fullfile(bucket.inFolder,'/Sigma_tau.mat'),'Sigma_tau');
    else
    load(fullfile(bucket.inFolder,'/Sigma_tau.mat'),'Sigma_tau');
end

