
%--------------------------------------------------------------------------
% JSI_experiments main
%--------------------------------------------------------------------------
bucket.pathToSubject = fullfile(bucket.datasetRoot, sprintf('S%02d',subjectID));
bucket.pathToTask    = fullfile(bucket.pathToSubject,sprintf('task%d',taskID));
bucket.pathToRawData = fullfile(bucket.pathToTask,'data');
bucket.pathToProcessedData   = fullfile(bucket.pathToTask,'processed');

disp(strcat('[Start] Analysis SUBJECT_ ',num2str(subjectID),', TRIAL_',num2str(taskID)'));

% Extraction of the masterFile
masterFile = load(fullfile(bucket.pathToRawData,sprintf(('S%02d_%02d.mat'),subjectID,taskID)));

% Option for computing the estimated Sigma
opts.Sigma_dgiveny = false;

% Define the template to be used
if opts.noC7joints
    addpath(genpath('templatesNoC7'));
    rmpath('templates'); %if exists
    disp('[Warning]: The following analysis will be done with C7joints locked/fixed in the models!');
else
    addpath(genpath('templates'));
    rmpath('templatesNoC7'); %if exists
end

%% ---------------------UNA TANTUM PROCEDURE-------------------------------
%% SUIT struct creation
if ~exist(fullfile(bucket.pathToProcessedData,'suit.mat'), 'file')
    disp('-------------------------------------------------------------------');
    disp(strcat('[Start] Suit extraction ...'));
    % 1) extract data from C++ parsed files
    extractSuitDataFromParsing;
    % 2) compute sensor position
    suit = computeSuitSensorPosition(suit);
    save(fullfile(bucket.pathToProcessedData,'suit.mat'),'suit');
    disp(strcat('[End] Suit extraction'));
else
    load(fullfile(bucket.pathToProcessedData,'suit.mat'));
end

%% IMPORTANT NOTE:
% The subjects performed the experimental tasks with the drill on the right
% hand. This code will be modified for taking into account the presence of
% the drill. URDF/OSIM models and IK computation will be affected
% by this change.

%% Extract subject parameters from SUIT
if ~exist(fullfile(bucket.pathToSubject,'subjectParamsFromData.mat'), 'file')
    subjectParamsFromData = subjectParamsComputation(suit, masterFile.Subject.Info.Weight);
    save(fullfile(bucket.pathToSubject,'subjectParamsFromData.mat'),'subjectParamsFromData');
else
    load(fullfile(bucket.pathToSubject,'subjectParamsFromData.mat'),'subjectParamsFromData');
end

if opts.EXO && opts.EXO_torqueLevelAnalysis
    % Add manually the mass of the exo (1.8 kg) on the pelvis:
    if ~exist(fullfile(bucket.pathToSubject,'subjectParamsFromDataEXO.mat'), 'file')
        subjectParamsFromDataEXO = subjectParamsFromData;
        subjectParamsFromDataEXO.pelvisMass = subjectParamsFromData.pelvisMass + 1.8;
        subjectParamsFromDataEXO.pelvisIxx  = (subjectParamsFromDataEXO.pelvisMass/12) * ...
            ((subjectParamsFromData.pelvisBox(2))^2 + (subjectParamsFromData.pelvisBox(3))^2);
        subjectParamsFromDataEXO.pelvisIyy  = (subjectParamsFromDataEXO.pelvisMass/12) * ...
            ((subjectParamsFromData.pelvisBox(3))^2 + (subjectParamsFromData.pelvisBox(1))^2);
        subjectParamsFromDataEXO.pelvisIzz  = (subjectParamsFromDataEXO.pelvisMass/12) * ...
            ((subjectParamsFromData.pelvisBox(3))^2 + (subjectParamsFromData.pelvisBox(2))^2);
        
        save(fullfile(bucket.pathToSubject,'subjectParamsFromDataEXO.mat'),'subjectParamsFromDataEXO');
    else
        load(fullfile(bucket.pathToSubject,'subjectParamsFromDataEXO.mat'),'subjectParamsFromDataEXO');
    end
end

%% Create URDF model
if opts.noC7joints
    % model NO exo, with C7 joints FIXED
    bucket.filenameURDF = fullfile(bucket.pathToSubject, sprintf('XSensURDF_subj%02d_48dof_noC7.urdf', subjectID));
    if ~exist(bucket.filenameURDF, 'file')
        bucket.URDFmodel = createXsensLikeURDFmodel(subjectParamsFromData, ...
            suit.sensors,...
            'filename',bucket.filenameURDF,...
            'GazeboModel',false);
    end
    % model WITH exo, with C7 joints FIXED
    if opts.EXO && opts.EXO_torqueLevelAnalysis
        bucket.filenameURDF = fullfile(bucket.pathToSubject, sprintf('XSensURDF_subj%02d_48dof_EXO_noC7.urdf', subjectID));
        if ~exist(bucket.filenameURDF, 'file')
            bucket.URDFmodel = createXsensLikeURDFmodel(subjectParamsFromDataEXO, ...
                suit.sensors,...
                'filename',bucket.filenameURDF,...
                'GazeboModel',false);
        end
    end
else
    % model NO exo  or EXO in forceLevelAnalysis, with C7 joints (complete) REVOLUTE
    bucket.filenameURDF = fullfile(bucket.pathToSubject, sprintf('XSensURDF_subj%02d_48dof.urdf', subjectID));
    if ~exist(bucket.filenameURDF, 'file')
        bucket.URDFmodel = createXsensLikeURDFmodel(subjectParamsFromData, ...
            suit.sensors,...
            'filename',bucket.filenameURDF,...
            'GazeboModel',false);
    end
    % model WITH exo in torqueLevelAnalysis, with C7 joints (complete) REVOLUTE
    if opts.EXO && opts.EXO_torqueLevelAnalysis
        bucket.filenameURDF = fullfile(bucket.pathToSubject, sprintf('XSensURDF_subj%02d_48dof_EXO.urdf', subjectID));
        if ~exist(bucket.filenameURDF, 'file')
            bucket.URDFmodel = createXsensLikeURDFmodel(subjectParamsFromDataEXO, ...
                suit.sensors,...
                'filename',bucket.filenameURDF,...
                'GazeboModel',false);
        end
    end
end

%% Create OSIM model
if opts.noC7joints
    % model NO exo, with C7 joints LOCKED
    bucket.filenameOSIM = fullfile(bucket.pathToSubject, sprintf('XSensOSIM_subj%02d_48dof_noC7.osim', subjectID));
    if ~exist(bucket.filenameOSIM, 'file')
        bucket.OSIMmodel = createXsensLikeOSIMmodel(subjectParamsFromData, ...
            bucket.filenameOSIM);
    end
    % model WITH exo, with C7 joints LOCKED
    if opts.EXO && opts.EXO_torqueLevelAnalysis
        bucket.filenameOSIM = fullfile(bucket.pathToSubject, sprintf('XSensOSIM_subj%02d_48dof_EXO_noC7.osim', subjectID));
        if ~exist(bucket.filenameOSIM, 'file')
            bucket.OSIMmodel = createXsensLikeOSIMmodel(subjectParamsFromDataEXO, ...
                bucket.filenameOSIM);
        end
    end
else
    % model NO exo or EXO in forceLevelAnalysis, with C7 joints (complete) UNLOCKED
    bucket.filenameOSIM = fullfile(bucket.pathToSubject, sprintf('XSensOSIM_subj%02d_48dof.osim', subjectID));
    if ~exist(bucket.filenameOSIM, 'file')
        bucket.OSIMmodel = createXsensLikeOSIMmodel(subjectParamsFromData, ...
            bucket.filenameOSIM);
    end
    % model WITH exo in torqueLevelAnalysis, with C7 joints (complete) UNLOCKED
    if opts.EXO && opts.EXO_torqueLevelAnalysis
        bucket.filenameOSIM = fullfile(bucket.pathToSubject, sprintf('XSensOSIM_subj%02d_48dof_EXO.osim', subjectID));
        if ~exist(bucket.filenameOSIM, 'file')
            bucket.OSIMmodel = createXsensLikeOSIMmodel(subjectParamsFromDataEXO, ...
                bucket.filenameOSIM);
        end
    end
end
%% Inverse Kinematic computation
if ~exist(fullfile(bucket.pathToProcessedData,'human_state_tmp.mat'), 'file')
    disp('-------------------------------------------------------------------');
    disp(strcat('[Start] IK computation ...'));
    bucket.setupFile = fullfile(pwd, 'templates', 'setupOpenSimIKTool_Template.xml');
    bucket.trcFile   = fullfile(bucket.pathToRawData,sprintf('S%02d_%02d.trc',subjectID,taskID));
    bucket.motFile   = fullfile(bucket.pathToProcessedData,sprintf('S%02d_%02d.mot',subjectID,taskID));
    [human_state_tmp, human_ddq_tmp, selectedJoints] = IK(bucket.filenameOSIM, ...
        bucket.trcFile, ...
        bucket.setupFile, ...
        suit.properties.frameRate, ...
        bucket.motFile);
    % here selectedJoints is the order of the Osim computation.
    save(fullfile(bucket.pathToProcessedData,'human_state_tmp.mat'),'human_state_tmp');
    save(fullfile(bucket.pathToProcessedData,'human_ddq_tmp.mat'),'human_ddq_tmp');
    save(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'),'selectedJoints');
    disp(strcat('[End] IK computation'));
else
    load(fullfile(bucket.pathToProcessedData,'human_state_tmp.mat'));
    load(fullfile(bucket.pathToProcessedData,'human_ddq_tmp.mat'));
    load(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'));
end
% disp('[Warning]: The IK is expressed in current frame and not in fixed frame!');

%% Raw data handling
rawDataHandling;

%% Save synchroData with the kinematics infos
if ~exist(fullfile(bucket.pathToProcessedData,'synchroKin.mat'), 'file')
    fieldsToBeRemoved = {'RightShoe_SF','LeftShoe_SF','FP_SF'};
    synchroKin = rmfield(synchroData,fieldsToBeRemoved);
    save(fullfile(bucket.pathToProcessedData,'synchroKin.mat'),'synchroKin');
end
load(fullfile(bucket.pathToProcessedData,'synchroKin.mat'));

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

%% Removal of C7 joints kinematics quantities
if opts.noC7joints
    if ~exist(fullfile(bucket.pathToProcessedData,'selectedJointsReduced.mat'), 'file')
        load(fullfile(bucket.pathToProcessedData,'synchroKin.mat'));
        synchroKinReduced = synchroKin;
        % Get the indices to be removed
        for sjIdx = 1 : size(selectedJoints,1)
            if (strcmp(selectedJoints{sjIdx,1},'jRightC7Shoulder_rotx'))
                jRshoC7Rotx_idx = sjIdx;
            end
        end
        selectedJoints(jRshoC7Rotx_idx,:) = [];
        
        for sjIdx = 1 : size(selectedJoints,1)
            if (strcmp(selectedJoints{sjIdx,1},'jLeftC7Shoulder_rotx'))
                jLshoC7Rotx_idx = sjIdx;
            end
        end
        selectedJoints(jLshoC7Rotx_idx,:) = [];
        
        selectedJointsReduced = selectedJoints;
        for blockIdx = 1 : block.nrOfBlocks
            synchroKinReduced(blockIdx).q(jRshoC7Rotx_idx,:) = [];
            synchroKinReduced(blockIdx).dq(jRshoC7Rotx_idx,:) = [];
            synchroKinReduced(blockIdx).ddq(jRshoC7Rotx_idx,:) = [];
        end
        for blockIdx = 1 : block.nrOfBlocks
            synchroKinReduced(blockIdx).q(jLshoC7Rotx_idx,:) = [];
            synchroKinReduced(blockIdx).dq(jLshoC7Rotx_idx,:) = [];
            synchroKinReduced(blockIdx).ddq(jLshoC7Rotx_idx,:) = [];
        end
        save(fullfile(bucket.pathToProcessedData,'selectedJointsReduced.mat'),'selectedJointsReduced');
        save(fullfile(bucket.pathToProcessedData,'synchroKinReduced.mat'),'synchroKinReduced');
    else
        load(fullfile(bucket.pathToProcessedData,'selectedJointsReduced.mat'));
        load(fullfile(bucket.pathToProcessedData,'synchroKinReduced.mat'));
    end
    
    % Overwrite old variables with the new reduced variables
    selectedJoints = selectedJointsReduced;
    save(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'),'selectedJoints');
    synchroKin = synchroKinReduced;
    save(fullfile(bucket.pathToProcessedData,'synchroKin.mat'),'synchroKin');
    % Remove useless quantities
    clearvars selectedJointsReduced synchroKinReduced;
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
% humanSensors.removeAllSensorsOfType(iDynTree.ACCELEROMETER_SENSOR);
bucket.base = 'Pelvis'; % floating base

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

% Floating base settings
currentBase = berdy.model().getLinkName(traversal.getBaseLink().getIndex());
disp(strcat('Current base is < ', currentBase,'>.'));
human_kinDynComp.setFloatingBase(currentBase);
baseKinDynModel = human_kinDynComp.getFloatingBase();
% Consistency check: berdy.model base and human_kinDynComp.model have to be consistent!
if currentBase ~= baseKinDynModel
    error(strcat('[ERROR] The berdy model base (',currentBerdyBase,') and the kinDyn model base (',baseKinDynModel,') do not match!'));
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

% ---------------------------------------------------
% CHECK: print the order of variables in d vector
% printBerdyDynVariables_floating(berdy)
% ---------------------------------------------------

%% Measurements wrapping
% Set the sensor covariance priors
priors = struct;
priors.acc_IMU     = 0.001111 * ones(3,1);           %[m^2/s^2], from datasheet
%priors.gyro_IMU    = 5*1e-5 * ones(3,1);             %[rad^2/s^2], from datasheet
priors.ddq         = 6.66e-6;                        %[rad^2/s^4], from worst case covariance
priors.foot_fext   = [59; 59; 36; 2.25; 2.25; 0.56]; %[N^2,(Nm)^2], from worst case covariance
priors.noSens_fext = 1e-6 * ones(6,1);               %[N^2,(Nm)^2]

disp('-------------------------------------------------------------------');
disp('[Start] Wrapping measurements...');
for blockIdx = 1 : block.nrOfBlocks
    fext.rightHuman = shoes(blockIdx).Right_HF;
    fext.leftHuman  = shoes(blockIdx).Left_HF;
    
    data(blockIdx).block = block.labels(blockIdx);
    data(blockIdx).data  = dataPackaging(blockIdx, ...
        humanModel,...
        humanSensors,...
        suit_runtime,...
        fext,...
        synchroKin(blockIdx).ddq,...
        bucket.contactLink, ...
        priors);
    % y vector as input for MAP
    [data(blockIdx).y, data(blockIdx).Sigmay] = berdyMeasurementsWrapping(berdy, ...
        data(blockIdx).data);
end
disp('[End] Wrapping measurements');
% ---------------------------------------------------
% CHECK: print the order of measurement in y
% printBerdySensorOrder(berdy);
% ---------------------------------------------------

%% ------------------------------- MAP ------------------------------------
%% Set MAP priors
priors.mud    = zeros(berdy.getNrOfDynamicVariables(), 1);
priors.Sigmad = 1e+4 * eye(berdy.getNrOfDynamicVariables());
priors.SigmaD = 1e-4 * eye(berdy.getNrOfDynamicEquations());

%% Possibility to remove a sensor from the analysis
% excluding the accelerometers and gyroscope for whose removal already
% exists the iDynTree option.
sensorsToBeRemoved = [];

% bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% bucket.temp.id = 'LeftHand';
% sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];
% % %
% bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% bucket.temp.id = 'RightHand';
% sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];
% % %
% % bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% % bucket.temp.id = 'RightFoot';
% % sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];
% %
% % bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% % bucket.temp.id = 'LeftFoot';
% % sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];

%% Compute the transformation of the base w.r.t. the global suit frame G
disp('-------------------------------------------------------------------');
disp(strcat('[Start] Computing the <',currentBase,'> iDynTree transform w.r.t. the global frame G...'));
%--------Computation of the suit base orientation and position w.r.t. G
for suitLinksIdx = 1 : size(suit.links,1)
    if suit.links{suitLinksIdx, 1}.label == currentBase
        basePos_tot  = suit.links{suitLinksIdx, 1}.meas.position;
        baseOrientation_tot = suit.links{suitLinksIdx, 1}.meas.orientation;
        break
    end
    break
end

for blockIdx = 1 : block.nrOfBlocks
    tmp.cutRange{blockIdx} = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
    bucket.basePosition(blockIdx).basePos_wrtG  = basePos_tot(:,tmp.cutRange{blockIdx});
    bucket.orientation(blockIdx).baseOrientation = baseOrientation_tot(:,tmp.cutRange{blockIdx});
end
clearvars basePos_tot baseOrientation_tot;

for blockIdx = 1 : block.nrOfBlocks
    G_T_base(blockIdx).block = block.labels(blockIdx);
    G_T_base(blockIdx).G_T_b = computeTransformBaseToGlobalFrame(human_kinDynComp, ...
        synchroKin(blockIdx),...
        bucket.orientation(blockIdx).baseOrientation, ...
        bucket.basePosition(blockIdx).basePos_wrtG);
end
disp(strcat('[End] Computing the <',currentBase,'> iDynTree transform w.r.t. the global frame G'));

%% Contact pattern definition
% Trials are performed with both the feet attached to the ground (i.e.,
% doubleSupport).  No single support is assumed for this analysis.
for blockIdx = 1 : block.nrOfBlocks
    contactPattern(blockIdx).block = block.labels(blockIdx);
    contactPattern(blockIdx).contactPattern = cell(length(synchroKin(blockIdx).masterTime),1);
    for tmpIdx = 1 : length(synchroKin(blockIdx).masterTime)
        contactPattern(blockIdx).contactPattern{tmpIdx} = 'doubleSupport';
    end
end

%% Velocity of the currentBase
% Code to handle the info of the velocity of the base.
% This value is mandatorily required in the floating-base formalism.

disp('-------------------------------------------------------------------');
disp(strcat('[Start] Computing the <',currentBase,'> velocity...'));
if ~exist(fullfile(bucket.pathToProcessedData,'baseVelocity.mat'), 'file')
    for blockIdx = 1 : block.nrOfBlocks
        baseVel(blockIdx).block = block.labels(blockIdx);
        [baseVel(blockIdx).baseLinVelocity, baseVel(blockIdx).baseAngVelocity] = computeBaseVelocity(human_kinDynComp, ...
            synchroKin(blockIdx),...
            G_T_base(blockIdx), ...
            contactPattern(blockIdx).contactPattern);
    end
    save(fullfile(bucket.pathToProcessedData,'baseVelocity.mat'),'baseVel');
else
    load(fullfile(bucket.pathToProcessedData,'baseVelocity.mat'));
end
disp(strcat('[End] Computing the <',currentBase,'> velocity'));


%% --------------------------- ID comparisons -----------------------------
% Section to benchmark MAP with
% - iDynTree::kinDynComputation::InverseDynamics()
% - iDynTree::ExtWrenchesAndJointTorquesEstimator::estimateExtWrenchesAndJointTorques()
if opts.MAPbenchmarking
    MAPbenchmarking;
end

%% MAP computation
if ~exist(fullfile(bucket.pathToProcessedData,'estimation.mat'), 'file')
    for blockIdx = 1 : block.nrOfBlocks
        priors.Sigmay = data(blockIdx).Sigmay;
        estimation(blockIdx).block = block.labels(blockIdx);
        if opts.Sigma_dgiveny
            disp('-------------------------------------------------------------------');
            disp(strcat('[Start] Complete MAP computation for Block ',num2str(blockIdx),'...'));
            [estimation(blockIdx).mu_dgiveny, estimation(blockIdx).Sigma_dgiveny] = MAPcomputation_floating(berdy, ...
                traversal, ...
                synchroKin(blockIdx),...
                data(blockIdx).y, ...
                priors, ...
                baseVel(blockIdx).baseAngVelocity, ...
                'SENSORS_TO_REMOVE', sensorsToBeRemoved);
            disp(strcat('[End] Complete MAP computation for Block ',num2str(blockIdx)));
            % TODO: variables extraction
            % Sigma_tau extraction from Sigma d --> since sigma d is very big, it
            % cannot be saved! therefore once computed it is necessary to extract data
            % related to tau and save that one!
            % TODO: extractSigmaOfEstimatedVariables
        else
            disp('-------------------------------------------------------------------');
            disp(strcat('[Start] mu_dgiveny MAP computation for Block ',num2str(blockIdx),'...'));
            [estimation(blockIdx).mu_dgiveny] = MAPcomputation_floating(berdy, ...
                traversal, ...
                synchroKin(blockIdx),...
                data(blockIdx).y, ...
                priors, ...
                baseVel(blockIdx).baseAngVelocity, ...
                'SENSORS_TO_REMOVE', sensorsToBeRemoved);
            disp(strcat('[End] mu_dgiveny MAP computation for Block ',num2str(blockIdx)));
        end
    end
    save(fullfile(bucket.pathToProcessedData,'estimation.mat'),'estimation');
else
    load(fullfile(bucket.pathToProcessedData,'estimation.mat'));
end

%% Variables extraction from MAP estimation
if ~exist(fullfile(bucket.pathToProcessedData,'estimatedVariables.mat'), 'file')
    % torque extraction via Berdy
    for blockIdx = 1 : block.nrOfBlocks
        disp('-------------------------------------------------------------------');
        disp(strcat('[Start] Torque extraction for Block ',num2str(blockIdx),'...'));
        estimatedVariables.tau(blockIdx).block  = block.labels(blockIdx);
        estimatedVariables.tau(blockIdx).label  = selectedJoints;
        estimatedVariables.tau(blockIdx).values = extractEstimatedTau_from_mu_dgiveny(berdy, ...
            estimation(blockIdx).mu_dgiveny, ...
            synchroKin(blockIdx).q);
        disp(strcat('[End] Torque extraction for Block ',num2str(blockIdx)));
    end
    % fext extraction, manual (no Berdy)
    for blockIdx = 1 : block.nrOfBlocks
        disp('-------------------------------------------------------------------');
        disp(strcat('[Start] External force extraction for Block ',num2str(blockIdx),'...'));
        estimatedVariables.Fext(blockIdx).block  = block.labels(blockIdx);
        estimatedVariables.Fext(blockIdx).label  = dVectorOrder;
        estimatedVariables.Fext(blockIdx).values = extractEstimatedFext_from_mu_dgiveny(berdy, ...
            dVectorOrder, ...
            estimation(blockIdx).mu_dgiveny);
        disp(strcat('[End] External force extraction for Block ',num2str(blockIdx)));
    end
    save(fullfile(bucket.pathToProcessedData,'estimatedVariables.mat'),'estimatedVariables');
else
    load(fullfile(bucket.pathToProcessedData,'estimatedVariables.mat'));
end

% if ~opts.EXO
%     % test (via plots) angles VS torques for the shoulders
%     plotShouldersAnglesVStorques;
% end

%% Simulated y
% This section is useful to compare the measurements in the y vector and
% the results of the MAP.  Note: you cannot compare directly the results of
% the MAP (i.e., mu_dgiveny) with the measurements in the y vector but you
% have to pass through the y_sim and only later to compare y and y_sim.
if ~exist(fullfile(bucket.pathToProcessedData,'y_sim.mat'), 'file')
    for blockIdx = 1 : block.nrOfBlocks
        disp('-------------------------------------------------------------------');
        disp(strcat('[Start] Simulated y computation for Block ',num2str(blockIdx),'...'));
        y_sim(blockIdx).block = block.labels(blockIdx);
        [y_sim(blockIdx).y_sim] = sim_y_floating(berdy, ...
            synchroKin(blockIdx),...
            traversal, ...
            baseVel(blockIdx).baseAngVelocity, ...
            estimation(blockIdx).mu_dgiveny);
        disp(strcat('[End] Simulated y computation for Block ',num2str(blockIdx)));
    end
    save(fullfile(bucket.pathToProcessedData,'y_sim.mat'),'y_sim');
else
    load(fullfile(bucket.pathToProcessedData,'y_sim.mat'));
end

%% Variables extraction from y_sim
if ~isfield(y_sim,'FextSim_RightFoot')
    extractFext_from_y_sim
end

%% ---------------------------- EXO ANALYSIS ------------------------------
if opts.EXO
    %% Load raw meas from EXO table
    loadEXOtableMeas;

    %% Change of coordinates (CoC)
    % Important note:
    % ---------------
    % This change of coordinates is related only to:
    % q_leftShoulder  &  tau_leftShoulder
    % q_rightShoulder &  tau_rightShoulder
    % ---------------
    changeOfCoordinates;
    save(fullfile(bucket.pathToProcessedData,'CoC.mat'),'CoC');
    
    % Extraction and round of the shoulder angle vectors
    for blockIdx = 1 : block.nrOfBlocks
        % right shoulder
        EXO.tmp.qToCompare_right = (- CoC(blockIdx).Rsho_qFirst(1,:) + 90)'; % operation to compare the angles: change sign and then +90 deg
        EXO.rightRoundedTable(blockIdx).block = block.labels(blockIdx);
        EXO.rightRoundedTable(blockIdx).qToCompare_right_round = round(EXO.tmp.qToCompare_right);
        
        % left shoulder
        EXO.tmp.qToCompare_left = (CoC(blockIdx).Lsho_qFirst(1,:) + 90)'; % operation to compare the angles: +90 deg
        EXO.leftRoundedTable(blockIdx).block = block.labels(blockIdx);
        EXO.leftRoundedTable(blockIdx).qToCompare_left_round = round(EXO.tmp.qToCompare_left);
    end
    
    % Extraction from table of values accordingly to the shoulder angle vectors (rounded_q)
    for blockIdx = 1 : block.nrOfBlocks
        % right
        for qIdx = 1 : size(EXO.rightRoundedTable(blockIdx).qToCompare_right_round,1)
            for tableIdx = 1 : size(EXO.extractedTable(1).shoulder_angles,1)
                if (EXO.rightRoundedTable(blockIdx).qToCompare_right_round(qIdx) == EXO.extractedTable(subjectID).shoulder_angles(tableIdx,1))
                    EXO.rightRoundedTable(blockIdx).F_arm_scher(qIdx)   = EXO.extractedTable(subjectID).F_arm_scher(tableIdx,1);
                    EXO.rightRoundedTable(blockIdx).F_arm_support(qIdx) = EXO.extractedTable(subjectID).F_arm_support(tableIdx,1);
                    % EXO.rightRoundedTable(blockIdx).F_ASkraft_x(qIdx)   = EXO.extractedTable(subjectID).F_ASkraft_x(tableIdx,1);
                    % EXO.rightRoundedTable(blockIdx).F_ASkraft_y(qIdx)   = EXO.extractedTable(subjectID).F_ASkraft_y(tableIdx,1);
                    EXO.rightRoundedTable(blockIdx).F_KGkraft_x(qIdx)   = EXO.extractedTable(subjectID).F_KGkraft_x(tableIdx,1);
                    EXO.rightRoundedTable(blockIdx).F_KGkraft_y(qIdx)   = EXO.extractedTable(subjectID).F_KGkraft_y(tableIdx,1);
                    EXO.rightRoundedTable(blockIdx).M_support_mod(qIdx) = EXO.extractedTable(subjectID).M_support_mod(tableIdx,1);
                end
            end
        end
        % left
        for qIdx = 1 : size(EXO.leftRoundedTable(blockIdx).qToCompare_left_round,1)
            for tableIdx = 1 : size(EXO.extractedTable(1).shoulder_angles,1)
                if (EXO.leftRoundedTable(blockIdx).qToCompare_left_round(qIdx) == EXO.extractedTable(subjectID).shoulder_angles(tableIdx,1))
                    EXO.leftRoundedTable(blockIdx).F_arm_scher(qIdx)   = EXO.extractedTable(subjectID).F_arm_scher(tableIdx,1);
                    EXO.leftRoundedTable(blockIdx).F_arm_support(qIdx) = EXO.extractedTable(subjectID).F_arm_support(tableIdx,1);
                    % EXO.leftRoundedTable(blockIdx).F_ASkraft_x(qIdx)   = EXO.extractedTable(subjectID).F_ASkraft_x(tableIdx,1);
                    % EXO.leftRoundedTable(blockIdx).F_ASkraft_y(qIdx)   = EXO.extractedTable(subjectID).F_ASkraft_y(tableIdx,1);
                    EXO.leftRoundedTable(blockIdx).F_KGkraft_x(qIdx)   = EXO.extractedTable(subjectID).F_KGkraft_x(tableIdx,1);
                    EXO.leftRoundedTable(blockIdx).F_KGkraft_y(qIdx)   = EXO.extractedTable(subjectID).F_KGkraft_y(tableIdx,1);
                    EXO.leftRoundedTable(blockIdx).M_support_mod(qIdx) = EXO.extractedTable(subjectID).M_support_mod(tableIdx,1);
                end
            end
        end
    end
    
    %% Torque level analysis
    if opts.EXO_torqueLevelAnalysis
        disp('-------------------------------------------------------------------');
        disp('[Start] EXO Torque level analysis');
        EXO_torqueLevelAnalysis;
        save(fullfile(bucket.pathToProcessedData,'exo_torqueLevel.mat'),'exo_tauLevel');
        disp('[End] EXO Torque level analysis');
    end
    
    %% Force level analysis
    if opts.EXO_forceLevelAnalysis
        disp('-------------------------------------------------------------------');
        disp('[Start] EXO Force level analysis');

        % -----------------------------------------------------------------
        % Compute the term for the feet implicit constraints
        % This section computes the term which makes the dynamics of the system to
        % satisfy the 2-feet contact constraint.
        disp('-------------------------------------------------------------------');
        disp(strcat('[Start] Computing the term for the feet implicit constraint...'));
        if ~exist(fullfile(bucket.pathToProcessedData,'implicitFeetContraint.mat'), 'file')
            for blockIdx = 1 : block.nrOfBlocks
                implFeetConstraint(blockIdx).block = block.labels(blockIdx);
                implFeetConstraint(blockIdx).term  = computeImplicitFeetConstraintForm(human_kinDynComp, ...
                    G_T_base(blockIdx), ...
                    synchroKin(blockIdx), ...
                    baseVel(blockIdx));
            end
            save(fullfile(bucket.pathToProcessedData,'implicitFeetContraint.mat'),'implFeetConstraint');
        else
            load(fullfile(bucket.pathToProcessedData,'implicitFeetContraint.mat'));
        end
        disp(strcat('[End] Computing the term for the feet implicit constraint'));
        % -----------------------------------------------------------------

        EXO_forceLevelAnalysis;
        save(fullfile(bucket.pathToProcessedData,'exo_forceLevel.mat'),'exo_forceLevel');
        disp('[End] EXO Force level analysis');
    end

    %% EXO inside MAP analysis
    if opts.EXO_insideMAP
        disp('-------------------------------------------------------------------');
        disp('[Start] EXO inside MAP analysis');
        if ~exist(fullfile(bucket.pathToProcessedData,'exo_insideMAP.mat'), 'file')
            % Call a parallel main to recompute the MAP estimation with the EXO
            % forces (properly transformed in human frames) into the y
            % measurements vector.
            main_EXOinsideMAP;
        else
            load(fullfile(bucket.pathToProcessedData,'exo_insideMAP.mat'));
        end
        disp('[End] EXO inside MAP analysis');
    end
end
disp('-------------------------------------------------------------------');
