
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
bucket.pathToSubject = sprintf(fullfile(pwd,'/dataJSI/S%02d'),subjectID);
bucket.pathToTask    = sprintf(fullfile(bucket.pathToSubject,'/task%d'),taskID);
bucket.pathToRawData = fullfile(bucket.pathToTask,'/data');
bucket.pathToProcessedData   = fullfile(bucket.pathToTask,'/processed');

% Options on the base formalism
opts.fixedBase = true;
opts.floatingBase = false;
if opts.fixedBase
    disp(strcat('Current formalism is fixed base.'));
else
    disp(strcat('Current formalism is floating base.'));
end

% Extraction of the masterFile
masterFile = load(fullfile(bucket.pathToRawData,sprintf(('S%02d_%02d.mat'),subjectID,taskID)));
% This file contains general information about each subject and
% all the sensors involved in the analysis:

%% ---------------------UNA TANTUM PROCEDURE-------------------------------
%% SUIT struct creation
if ~exist(fullfile(bucket.pathToProcessedData,'suit.mat'))
    % 1) extract data from C++ parsed files
    extractSuitDataFromParsing;
    % 2) compute sensor position
    suit = computeSuitSensorPosition(suit);
    save(fullfile(bucket.pathToProcessedData,'/suit.mat'),'suit');
else
    load(fullfile(bucket.pathToProcessedData,'suit.mat'));
end

%% Extract subject parameters from SUIT
subjectParamsFromData = subjectParamsComputation(suit, masterFile.Subject.Info.Weight);

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
% if opts.floatingBase
%     bucket.inFolder = fullfile(bucket.pathToProcessedData,'/floating');
% end
if ~exist(fullfile(bucket.pathToProcessedData,'human_state_tmp.mat'))
    bucket.setupFile = fullfile(pwd,'/dataJSI/fileSetup.xml');
    bucket.trcFile = fullfile(bucket.pathToRawData,sprintf(('S%02d_%02d.trc'),subjectID,taskID));
    [human_state_tmp, human_ddq_tmp, selectedJoints] = IK(bucket.filenameOSIM, ...
                                                          bucket.trcFile, ...
                                                          bucket.setupFile);
    % here selectedJoints is the order of the Osim computation.
    save(fullfile(bucket.pathToProcessedData,'/human_state_tmp.mat'),'human_state_tmp');
    save(fullfile(bucket.pathToProcessedData,'/human_ddq_tmp.mat'),'human_ddq_tmp');
    save(fullfile(bucket.pathToProcessedData,'/selectedJoints.mat'),'selectedJoints');
else
    load(fullfile(bucket.pathToProcessedData,'/human_state_tmp.mat'),'human_state_tmp');
    load(fullfile(bucket.pathToProcessedData,'/human_ddq_tmp.mat'),'human_ddq_tmp');
    load(fullfile(bucket.pathToProcessedData,'/selectedJoints.mat'),'selectedJoints');
end

%% Define force data modality (shoes or forceplates)
shoes_bool              = true;     % if true --> shoes + Xsens
forceplates_bool        = false;    % if true --> forceplates + Xsens
shoesVSforceplates_bool = false;    % if true --> shoes + forceplates for the comparison

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
if opts.floatingBase
    base = 'RightFoot';
else
    % Remove sensor on the fixed base
    base = 'RightFoot';
    humanSensors.removeSensor(iDynTree.ACCELEROMETER_SENSOR, strcat(base,'_accelerometer'));
%     humanSensors.removeSensor(iDynTree.GYROSCOPE_SENSOR, strcat(base,'_gyro'));
    % We decided to remove gyroscopes from the analysis
    % humanSensors.removeAllSensorsOfType(iDynTree.ACCELEROMETER_SENSOR);
end

%% Initialize berdy
% Specify berdy options
berdyOptions = iDynTree.BerdyOptions;

berdyOptions.baseLink = base;
berdyOptions.includeAllNetExternalWrenchesAsSensors          = true;
berdyOptions.includeAllNetExternalWrenchesAsDynamicVariables = true;
berdyOptions.includeAllJointAccelerationsAsSensors           = true;
berdyOptions.includeAllJointTorquesAsSensors                 = false;

if opts.floatingBase
    berdyOptions.berdyVariant = iDynTree.BERDY_FLOATING_BASE;
    berdyOptions.includeFixedBaseExternalWrench = false;
else
    %----------------------------------------------------------------
    % IMPORTANT NOTE:
    % ---------------
    % Until this point the base link was 'Pelvis' but from now on  we
    % decided to change it with the 'LeftFoot' since it is really fixed
    % during the experiment.
    %----------------------------------------------------------------
    berdyOptions.includeFixedBaseExternalWrench = true;
end

% Load berdy
berdy = iDynTree.BerdyHelper;
berdy.init(humanModel, humanSensors, berdyOptions);
% Get the current traversal
traversal = berdy.dynamicTraversal;
currentBase = berdy.model().getLinkName(traversal.getBaseLink().getIndex());
disp(strcat('Current base is < ', currentBase,'>.'));
if opts.fixedBase
   disp(strcat('Be sure that sensors in URDF related to <', currentBase,'> has been removed!'));
end
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

if opts.floatingBase
    % ---------------------------------------------------
    % CHECK: print the order of variables in d vector
%     printBerdyDynVariables_floating(berdy)
    % ---------------------------------------------------
else
    % ---------------------------------------------------
    % CHECK: print the order of variables in d vector
%     printBerdyDynVariables(berdy)
    % ---------------------------------------------------
end
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

% % test 3: to constrain the accelerometer on the LeftFoot to be [0 0 9.81]
% % with a small associated variance.
% data(17).meas = repmat([0;0;9.81],1,size(data(17).meas,2));
% data(17).var = 1e-9 * data(17).var;

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
% sensorsToBeRemoved = [];
% temp = struct;
% temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% temp.id = 'LeftHand';
% sensorsToBeRemoved = [sensorsToBeRemoved; temp];
%
% temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% temp.id = 'RightHand';
% sensorsToBeRemoved = [sensorsToBeRemoved; temp];

% temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% temp.id = 'RightFoot';
% sensorsToBeRemoved = [sensorsToBeRemoved; temp];
%
% temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% temp.id = 'LeftFoot';
% sensorsToBeRemoved = [sensorsToBeRemoved; temp];


if opts.floatingBase
    %--------------------------------------------------------
    % we need to extract the angular velocity related to the chosen
    % current base and to use it into the MAP computation
    for i = 1 : length(suit_downsampled.sensors)
        if strcmp(suit_downsampled.sensors{i, 1}.label, currentBase)
            baseAngVel = suit_downsampled.sensors{i, 1}.meas.sensorAngularVelocity;
            break;
        end
    end
    %--------------------------------------------------------
    % mu_dgiveny computation
    if ~exist(fullfile(bucket.inFolder,'mu_dgiveny.mat'))
        [mu_dgiveny, ~] = MAPcomputation_floating(berdy, traversal, human_state, y, priors, baseAngVel);
        save(fullfile(bucket.inFolder,'/mu_dgiveny.mat'),'mu_dgiveny');
        else
        load(fullfile(bucket.inFolder,'/mu_dgiveny.mat'),'mu_dgiveny');
    end
    %--------------------------------------------------------
    % variables extraction
    estimated_variables = estimatedVariablesExtraction_floating(berdy, mu_dgiveny);
    save(fullfile(bucket.inFolder,'/estimated_variables.mat'),'estimated_variables');
    %--------------------------------------------------------
    % Sigma_tau extraction from Sigma d --> since sigma d is very big, it
    % cannot be saved! therefore once computed it is necessary to extract data
    % related to tau and save that one!
     if ~exist(fullfile(bucket.inFolder,'Sigma_tau.mat'))
        [~, Sigma_dgiveny] = MAPcomputation_floating(berdy, human_state, y, priors);
        extractSigmaOfEstimatedVariables
        save(fullfile(bucket.inFolder,'/Sigma_tau.mat'),'Sigma_tau');
        else
        load(fullfile(bucket.inFolder,'/Sigma_tau.mat'),'Sigma_tau');
    end
end

if opts.fixedBase
    %--------------------------------------------------------
    % mu_dgiveny computation
    if ~exist(fullfile(bucket.inFolder,'mu_dgiveny.mat'))
        [mu_dgiveny, ~] = MAPcomputation(berdy, human_state, y, priors);
        save(fullfile(bucket.inFolder,'/mu_dgiveny.mat'),'mu_dgiveny');
        else
        load(fullfile(bucket.inFolder,'/mu_dgiveny.mat'),'mu_dgiveny');
    end
    %--------------------------------------------------------
    % variables extraction
    estimated_variables = estimatedVariablesExtraction(berdy, currentBase, mu_dgiveny);
    save(fullfile(bucket.inFolder,'/estimated_variables.mat'),'estimated_variables');
    %--------------------------------------------------------
    % Sigma_tau extraction from Sigma d --> since sigma d is very big, it
    % cannot be saved! therefore once computed it is necessary to extract data
    % related to tau and save that one!
    if ~exist(fullfile(bucket.inFolder,'Sigma_tau.mat'))
        [~, Sigma_dgiveny] = MAPcomputation(berdy, human_state, y, priors);
        extractSigmaOfEstimatedVariables
        save(fullfile(bucket.inFolder,'/Sigma_tau.mat'),'Sigma_tau');
        else
        load(fullfile(bucket.inFolder,'/Sigma_tau.mat'),'Sigma_tau');
    end
end

%% testing

TEST_FLOATING_compareVariables;

