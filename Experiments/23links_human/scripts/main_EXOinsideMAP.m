
%% Transform forces from the EXO into human forces
disp('-------------------------------------------------------------------');
disp('       [Start] Transforming EXO force in human frames...');
if ~exist(fullfile(bucket.pathToProcessedData,'EXOfext.mat'), 'file')
    transformEXOforcesInHumanFrames;
else
    load(fullfile(bucket.pathToProcessedData,'EXOfext.mat'));
end
disp('       [End] Transforming EXO force in human frames');

%% Wrapping new measurements with EXO in y vector
disp('-------------------------------------------------------------------');
disp('       [Start] Wrapping measurements with EXO in y...');
priors.exo_fext   = 1e1 * ones(6,1); %[N^2,(Nm)^2]

% Find links where EXO forces are acting
lenCheck = length(data(blockIdx).data);
for idIdx = 1 : lenCheck
    if strcmp(data(1).data(idIdx).id,'Pelvis')
        tmp.pelvisIdx = idIdx;
    end
    if strcmp(data(1).data(idIdx).id,'LeftUpperArm')
        tmp.LUAIdx = idIdx;
    end
    if strcmp(data(1).data(idIdx).id,'RightUpperArm')
        tmp.RUAIdx = idIdx;
    end
end

% Create new data packaging with EXO forces
for blockIdx = 1 : block.nrOfBlocks
    dataWithEXOiny(blockIdx).block = data(blockIdx).block;
    dataWithEXOiny(blockIdx).data = data(blockIdx).data;
    
    % Pelvis
    dataWithEXOiny(blockIdx).data(tmp.pelvisIdx).meas = EXOfext(blockIdx).PELVIS;
    dataWithEXOiny(blockIdx).data(tmp.pelvisIdx).var  = priors.exo_fext;
    % LUA
    dataWithEXOiny(blockIdx).data(tmp.LUAIdx).meas = EXOfext(blockIdx).LUA;
    dataWithEXOiny(blockIdx).data(tmp.LUAIdx).var  = priors.exo_fext;
    % RUA
    dataWithEXOiny(blockIdx).data(tmp.RUAIdx).meas = EXOfext(blockIdx).RUA;
    dataWithEXOiny(blockIdx).data(tmp.RUAIdx).var  = priors.exo_fext;

    % y vector with EXO forces as input for MAP
    [dataWithEXOiny(blockIdx).y, dataWithEXOiny(blockIdx).Sigmay] = berdyMeasurementsWrapping(berdy, ...
        dataWithEXOiny(blockIdx).data);
end
disp('       [End] Wrapping measurements with EXO in y');
% ---------------------------------------------------
% CHECK: print the order of measurement in y
% printBerdySensorOrder(berdy);
% ---------------------------------------------------

%% MAP computation with EXO in y
for blockIdx = 1 : block.nrOfBlocks
    priors.Sigmay = dataWithEXOiny(blockIdx).Sigmay;
    estimationWithEXOiny(blockIdx).block = block.labels(blockIdx);
    if opts.Sigma_dgiveny
        disp('-------------------------------------------------------------------');
        disp(strcat('       [Start] Complete MAP computation with EXO in y for Block ',num2str(blockIdx),'...'));
        [estimationWithEXOiny(blockIdx).mu_dgiveny, estimationWithEXOiny(blockIdx).Sigma_dgiveny] = MAPcomputation_floating(berdy, ...
            traversal, ...
            synchroKin(blockIdx),...
            dataWithEXOiny(blockIdx).y, ...
            priors, ...
            baseVel(blockIdx).baseAngVelocity, ...
            'SENSORS_TO_REMOVE', sensorsToBeRemoved);
        disp(strcat('       [End] Complete MAP computation with EXO in y for Block ',num2str(blockIdx)));
        % TODO: variables extraction
        % Sigma_tau extraction from Sigma d --> since sigma d is very big, it
        % cannot be saved! therefore once computed it is necessary to extract data
        % related to tau and save that one!
        % TODO: extractSigmaOfEstimatedVariables
    else
        disp('-------------------------------------------------------------------');
        disp(strcat('       [Start] mu_dgiveny MAP computation with EXO in y for Block ',num2str(blockIdx),'...'));
        [estimationWithEXOiny(blockIdx).mu_dgiveny] = MAPcomputation_floating(berdy, ...
            traversal, ...
            synchroKin(blockIdx),...
            dataWithEXOiny(blockIdx).y, ...
            priors, ...
            baseVel(blockIdx).baseAngVelocity, ...
            'SENSORS_TO_REMOVE', sensorsToBeRemoved);
        disp(strcat('       [End] mu_dgiveny MAP computation with EXO in y for Block ',num2str(blockIdx)));
    end
end

%% Variables extraction from MAP estimation with EXO in y
% torque extraction via Berdy
for blockIdx = 1 : block.nrOfBlocks
    disp('-------------------------------------------------------------------');
    disp(strcat('       [Start] Torque extraction with EXO in y for Block ',num2str(blockIdx),'...'));
    estimatedVariablesWithEXOiny.tau(blockIdx).block  = block.labels(blockIdx);
    estimatedVariablesWithEXOiny.tau(blockIdx).label  = selectedJoints;
    estimatedVariablesWithEXOiny.tau(blockIdx).values = extractEstimatedTau_from_mu_dgiveny(berdy, ...
        estimationWithEXOiny(blockIdx).mu_dgiveny, ...
        synchroKin(blockIdx).q);
    disp(strcat('       [End] Torque extraction with EXO in y for Block ',num2str(blockIdx)));
end
% fext extraction, manual (no Berdy)
for blockIdx = 1 : block.nrOfBlocks
    disp('-------------------------------------------------------------------');
    disp(strcat('       [Start] External force extraction with EXO in y for Block ',num2str(blockIdx),'...'));
    estimatedVariablesWithEXOiny.Fext(blockIdx).block  = block.labels(blockIdx);
    estimatedVariablesWithEXOiny.Fext(blockIdx).label  = dVectorOrder;
    estimatedVariablesWithEXOiny.Fext(blockIdx).values = extractEstimatedFext_from_mu_dgiveny(berdy, ...
        dVectorOrder, ...
        estimationWithEXOiny(blockIdx).mu_dgiveny);
    disp(strcat('       [End] External force extraction with EXO in y for Block ',num2str(blockIdx)));
end

%% Simulated y with EXO in y
% This section is useful to compare the measurements in the y vector and
% the results of the MAP.  Note: you cannot compare directly the results of
% the MAP (i.e., mu_dgiveny) with the measurements in the y vector but you
% have to pass through the y_sim and only later to compare y and y_sim.
for blockIdx = 1 : block.nrOfBlocks
    disp('-------------------------------------------------------------------');
    disp(strcat('       [Start] Simulated y computation with EXO in y for Block ',num2str(blockIdx),'...'));
    y_simWithEXOiny(blockIdx).block = block.labels(blockIdx);
    [y_simWithEXOiny(blockIdx).y_sim] = sim_y_floating(berdy, ...
        synchroKin(blockIdx),...
        traversal, ...
        baseVel(blockIdx).baseAngVelocity, ...
        estimationWithEXOiny(blockIdx).mu_dgiveny);
    disp(strcat('       [End] Simulated y computation with EXO in y for Block ',num2str(blockIdx)));
end

%% Cluster results in a generic struct
exo_insideMAP = struct;
exo_insideMAP.estimation         = estimationWithEXOiny;
exo_insideMAP.estimatedVariables = estimatedVariablesWithEXOiny;
exo_insideMAP.y_sim              = y_simWithEXOiny;
save(fullfile(bucket.pathToProcessedData,'exo_insideMAP.mat'),'exo_insideMAP');
disp('-------------------------------------------------------------------');
