
%--------------------------------------------------------------------------
% Experiment main
%--------------------------------------------------------------------------

% Path to the folder of the subject and the task, respectively
bucket.pathToSubject = fullfile(bucket.datasetRoot, sprintf('S%02d',subjectID));
bucket.pathToTask    = fullfile(bucket.pathToSubject,sprintf('task%02d',taskID));

% Path to the folder for raw data and URDFs.
bucket.pathToWearableData = fullfile(bucket.pathToTask,'data/wearable');
bucket.pathToIKdata       = fullfile(bucket.pathToTask,'data/humanState');
bucket.pathToURDF         = fullfile(bucket.pathToSubject,'URDFs');

% Path to the folder where processed data will be saved
bucket.pathToProcessedData   = fullfile(bucket.pathToTask,'processed');
if ~exist(bucket.pathToProcessedData)
    mkdir (bucket.pathToProcessedData)
end

disp(strcat('[Start] Analysis SUBJECT_ ',num2str(subjectID),', TRIAL_',num2str(taskID)'));

%% URDF loading
if opts.analysis_48dofURDF
    nrDofs = 48;
    bucket.URDFfilename = fullfile(bucket.pathToURDF,sprintf('humanSubject%02d_%ddof.urdf',subjectID,nrDofs));
end

if opts.analysis_66dofURDF
    nrDofs = 66;
    bucket.URDFfilename = fullfile(bucket.pathToURDF,sprintf('humanSubject%02d_%ddof.urdf',subjectID,nrDofs));
end

%% SUIT struct creation
if ~exist(fullfile(bucket.pathToProcessedData,'suit.mat'), 'file')
    disp('-------------------------------------------------------------------');
    disp('[Start] Suit extraction ...');
    % 1) ---extract data from suit as YARP-dumped IWear file
    extractWearableDataFromIWear;
    % Change the name of T4Shoulder in C7Shoulder
    for jointsIdx = 1 : wearData.properties.nrOfJoints
        if strcmp(wearData.joints{jointsIdx, 1}.label,'jLeftT4Shoulder')
            wearData.joints{jointsIdx, 1}.label = 'jLeftC7Shoulder';
        end
        if strcmp(wearData.joints{jointsIdx, 1}.label,'jRightT4Shoulder')
            wearData.joints{jointsIdx, 1}.label = 'jRightC7Shoulder';
        end
    end

    % 2) ---compute sensor position w.r.t. the links
    disp('[Warning]: Check manually the length of the data for the sensor position computation!');
    disp('[Warning]: By default, the computation is done by considering all the samples. It may take time!');
    suit = computeSuitSensorPosition(wearData, wearData.nrOfFrames);
    save(fullfile(bucket.pathToProcessedData,'suit.mat'),'suit');
    disp('[End] Suit extraction');
else
    load(fullfile(bucket.pathToProcessedData,'suit.mat'));
end

%% Transform the sensorFreeAcceleration
% Code to transform the <suit.sensors.sensorFreeAcceleration> (i.e., the sensor
% acceleration without the gravity, expressed w.r.t. the Xsens global frame G)
% into <suit.sensors.sensorOldAcceleration> (i.e., the sensor
% acceleration with the gravity, expressed w.r.t. the sensor frame)

if ~isfield(suit.sensors{1, 1}.meas,'sensorOldAcceleration')
    gravity = [0; 0; -9.81];
    for sensIdx = 1: suit.properties.nrOfSensors
        for lenIdx = 1 : suit.nrOfFrames
            G_R_S = quat2Mat(suit.sensors{sensIdx, 1}.meas.sensorOrientation(:,lenIdx));
            % Transformation:        S_a_old = S_R_G * (G_a_new - gravity)
            suit.sensors{sensIdx, 1}.meas.sensorOldAcceleration(:,lenIdx) = ...
                transpose(G_R_S) * ...
                (suit.sensors{sensIdx, 1}.meas.sensorFreeAcceleration(:,lenIdx) - gravity);
        end
    end
    save(fullfile(bucket.pathToProcessedData,'suit.mat'),'suit');
else
    load(fullfile(bucket.pathToProcessedData,'suit.mat'));
end

%% IK struct creation
disp('-------------------------------------------------------------------');
disp('[Start] IK analysis ...');
extractIKfromHumanStateProvider;

% Frame rate consistency check
if suit.estimatedFrameRate ~= IKdata.estimatedFrameRate
    error('The frame rates of suit and IKdata are different! Check it! ...')
end

% Number of samples/timestamps consistency check
if suit.nrOfFrames ~= IKdata.nrOfFrames
    disp('[Info] The number of frames of suit and IKdata is different.')
    timestampAndNrSamplesEquilizer;
end

% Rearrange q in the required format:
synchroKin.label   = cell(nrDofs,1);
synchroKin.state.q = [ ];
for jointsIdx = 1: nrDofs
    synchroKin.label{jointsIdx,1} =  IKdata.joints{jointsIdx, 1}.label;
    synchroKin.state.q = [synchroKin.state.q, IKdata.joints{jointsIdx, 1}.angle'];
end
synchroKin.state.q = synchroKin.state.q';

%% Computation of dq,ddq via Savitzi-Golay
% Set Sg parameters
Sg.samplingTime = 1/suit.estimatedFrameRate;
Sg.polinomialOrder = 3;
Sg.window = 5; % required by the moving-window avarage filter.

[~,synchroKin.state.dq,synchroKin.ddq] = SgolayFilterAndDifferentiation(Sg.polinomialOrder, ...
    Sg.window,synchroKin.state.q,Sg.samplingTime); %in rad
disp('[End] IK analysis');

%% Transform feet forces from sensor frames into human frames
% 1.Manual transform computation

bucket.linkInShoes = cell(2,1);
bucket.linkInShoes{1} = 'RightFoot'; % human link in contact with RightShoe
bucket.linkInShoes{2} = 'LeftFoot';  % human link in contact with LeftShoe

shoesForcesInSensorFrames = struct;
shoesForcesInSensorFrames.LeftShoe_SF  = suit.ftShoes.Left';
shoesForcesInSensorFrames.RightShoe_SF = suit.ftShoes.Right';

shoes = transformShoesWrenches(shoesForcesInSensorFrames);

% 2.Import transform from IHumanForceProvider
% TODO: the comparison.

%% ------------------------RUNTIME PROCEDURE-------------------------------
%% Load URDF model with sensors
humanModel.filename = bucket.URDFfilename;
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
disp('-------------------------------------------------------------------');
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
disp(strcat('[Info] Current base is < ', currentBase,'>.'));
human_kinDynComp.setFloatingBase(currentBase);
baseKinDynModel = human_kinDynComp.getFloatingBase();

% Consistency check: berdy.model base and human_kinDynComp.model have to be consistent!
if currentBase ~= baseKinDynModel
    error(strcat('The berdy model base (',currentBase,') and the kinDyn model base (',baseKinDynModel,') do not match!'));
end

% Get the tree is visited as the order of variables in vector d
dVectorOrder = cell(traversal.getNrOfVisitedLinks(), 1);
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
% Set the sensor covariance priors (to be tailored by the user)
priors = struct;
priors.acc_IMU     = 0.001111 * ones(3,1);                  %[m^2/s^2]   , from datasheet
% priors.gyro_IMU    = xxxxxx * ones(3,1);                  %[rad^2/s^2] , from datasheet
priors.ddq         = 6.66e-6;                               %[rad^2/s^4] , from worst case covariance
priors.foot_fext   = 1e-3 * [59; 59; 36; 2.25; 2.25; 0.56]; %[N^2,(Nm)^2]
priors.noSens_fext = 1e-6 * ones(6,1);                      %[N^2,(Nm)^2]

disp('-------------------------------------------------------------------');
disp('[Start] Wrapping measurements...');
fext.rightHuman = shoes.Right_HF;
fext.leftHuman  = shoes.Left_HF;

data = dataPackaging(humanModel, ...
    humanSensors, ...
    suit, ...
    fext, ...
    synchroKin.ddq, ...
    bucket.linkInShoes, ...
    priors);
% y vector as input for MAP
[y, Sigmay] = berdyMeasurementsWrapping(berdy, data);
disp('[End] Wrapping measurements');

% ---------------------------------------------------
% CHECK: print the order of measurement in y
% printBerdySensorOrder(berdy);
% ---------------------------------------------------

%% Compute the transformation of the base w.r.t. the global suit frame G
disp('-------------------------------------------------------------------');
disp(strcat('[Start] Computing the <',currentBase,'> iDynTree transform w.r.t. the global frame G...'));
%--------Computation of the suit base orientation and position w.r.t. G
for suitLinksIdx = 1 : size(suit.links,1)
    if strcmp(suit.links{suitLinksIdx, 1}.label, currentBase)
        basePos_wrtG  = suit.links{suitLinksIdx, 1}.meas.position;
        baseOrientation = suit.links{suitLinksIdx, 1}.meas.orientation;
    end
end

G_T_b = computeTransformBaseToGlobalFrame(human_kinDynComp, synchroKin.state,...
    baseOrientation, basePos_wrtG);

disp(strcat('[End] Computing the <',currentBase,'> iDynTree transform w.r.t. the global frame G'));

%% Contact pattern analysis
patternRanges.parameterForDStuning = 19;
% This value is a % of the mean total weight of the subject.  It defines how
% big the area for double support (DS) has to be considered in the
% analysis.

patternRanges.sampleDSTreshold = 20;
% For a number of samples < patternRanges.sampleDSTreshold, DS is
% considered as the previous SS.

contactPatternDetection;

%% Computation of the angular velocity of the currentBase
% The angular velocity of the base is mandatorily required in the
% floating-base formalism.
disp('-------------------------------------------------------------------');
disp(strcat('[Start] Computing the <',currentBase,'> velocity...'));
[baseVelocity.linear, baseVelocity.angular] = computeBaseVelocity(human_kinDynComp, ...
    synchroKin.state, G_T_b, contactPattern);
disp(strcat('[End] Computing the <',currentBase,'> velocity'));

%% ------------------------------- MAP ------------------------------------
%% Set MAP priors
priors.mud    = zeros(berdy.getNrOfDynamicVariables(), 1);
priors.Sigmad = 1e+4 * eye(berdy.getNrOfDynamicVariables());
% 1e+4 means low reliability on the estimation (i.e., no prior info on the final solution d)
priors.SigmaD = 1e-4 * eye(berdy.getNrOfDynamicEquations());
% 1e-4 means high reliability on the model constraints

%% Possibility to remove a sensor from the analysis
% except fot the accelerometers and gyroscope for whose removal already
% exists the iDynTree option.

sensorsToBeRemoved = [];

% %-----HOW TO REMOVE WRENCH SENSORS FROM THE HANDS
% bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% bucket.temp.id = 'LeftHand';
% sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];
%
% bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% bucket.temp.id = 'RightHand';
% sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];

% %-----HOW TO REMOVE WRENCH SENSORS FROM THE FEET
% bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% bucket.temp.id = 'RightFoot';
% sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];
%
% bucket.temp.type = iDynTree.NET_EXT_WRENCH_SENSOR;
% bucket.temp.id = 'LeftFoot';
% sensorsToBeRemoved = [sensorsToBeRemoved; bucket.temp];

%% MAP computation
disp('-------------------------------------------------------------------');
if ~exist(fullfile(bucket.pathToProcessedData,'estimation.mat'), 'file')
    priors.Sigmay = Sigmay;
    if opts.Sigma_dgiveny
        disp('[Start] Complete MAP computation...');
        [estimation.mu_dgiveny, estimation.Sigma_dgiveny] = MAPcomputation_floating(berdy, ...
            traversal, ...
            synchroKin.state, ...
            y, ...
            priors, ...
            baseVelocity.angular, ...
            'SENSORS_TO_REMOVE', sensorsToBeRemoved);
        disp('[End] Complete MAP computation');
        % TODO: variables extraction
        % Sigma_tau extraction from Sigma d --> since Sigma d is very big, it
        % cannot be saved! Therefore once computed it is necessary to extract data
        % related to tau and save that one!
        % TODO: extractSigmaOfEstimatedVariables
    else
        disp('[Start] mu_dgiveny MAP computation...');
        [estimation.mu_dgiveny] = MAPcomputation_floating(berdy, ...
            traversal, ...
            synchroKin.state,...
            y, ...
            priors, ...
            baseVelocity.angular, ...
            'SENSORS_TO_REMOVE', sensorsToBeRemoved);
        disp('[End] mu_dgiveny MAP computation');
    end
    
    save(fullfile(bucket.pathToProcessedData,'estimation.mat'),'estimation');
else
    disp('MAP computation already saved!');
    load(fullfile(bucket.pathToProcessedData,'estimation.mat'));
end

%% Variables extraction from MAP estimation
disp('-------------------------------------------------------------------');
if ~exist(fullfile(bucket.pathToProcessedData,'estimatedVariables.mat'), 'file')
    % torque extraction (via Berdy)
    disp('[Start] Torque MAP extraction...');
    estimatedVariables.tau.label  = selectedJoints;
    estimatedVariables.tau.values = extractEstimatedTau_from_mu_dgiveny(berdy, ...
        estimation.mu_dgiveny, ...
        synchroKin.state.q);
    disp('[End] Torque MAP extraction');
    
    % fext extraction (no via Berdy)
    disp('-------------------------------------------------------------------');
    disp('[Start] External force MAP extraction...');
    estimatedVariables.Fext.label  = dVectorOrder;
    estimatedVariables.Fext.values = extractEstimatedFext_from_mu_dgiveny(berdy, ...
        dVectorOrder, ...
        estimation.mu_dgiveny);
    disp('[End] External force MAP extraction for Block');
    
    % save extracted viariables
    save(fullfile(bucket.pathToProcessedData,'estimatedVariables.mat'),'estimatedVariables');
else
    disp('Torque and ext force MAP extraction already saved!');
    load(fullfile(bucket.pathToProcessedData,'estimatedVariables.mat'));
end

%% Simulated y
% This section is useful to compare the measurements in the y vector and
% the results of the MAP.  Note: you cannot compare directly the results of
% the MAP (i.e., mu_dgiveny) with the measurements in the y vector but you
% have to pass through the y_sim and only later to compare y and y_sim.
disp('-------------------------------------------------------------------');
if ~exist(fullfile(bucket.pathToProcessedData,'y_sim.mat'), 'file')
    disp('[Start] Simulated y computation...');
    [y_sim] = sim_y_floating(berdy, ...
        synchroKin.state, ...
        traversal, ...
        baseVelocity.angular, ...
        estimation.mu_dgiveny);
    disp('[End] Simulated y computation');
    save(fullfile(bucket.pathToProcessedData,'y_sim.mat'),'y_sim');
else
    disp('Simulated y computation already saved!');
    load(fullfile(bucket.pathToProcessedData,'y_sim.mat'));
end

%% Variables extraction from y_sim
disp('-------------------------------------------------------------------');
if ~exist(fullfile(bucket.pathToProcessedData,'y_sim_fext.mat'), 'file')
    disp('[Start] Simulated y forces extraction...');
    extractFext_from_y_sim;
    disp('[End] Simulated y forces extraction');
else
    disp('Simulated y forces extraction already saved!');
    load(fullfile(bucket.pathToProcessedData,'y_sim_fext.mat'));
end
