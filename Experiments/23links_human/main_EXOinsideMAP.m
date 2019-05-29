
%% Transform forces from the EXO into human forces
% - 1 force on the LeftUpperArm
% - 1 force on the RightUpperArm
% - 1 force at the Pelvis (sum of the two exo forces oat the hips)
disp('-------------------------------------------------------------------');
disp('[Start] Transforming EXO force in human frames...');
% TODO: transformEXOforcesInHumanFrames;
disp('[End] Transforming EXO force in human frames');

%% Wrapping new measurements with EXO in y vector
disp('-------------------------------------------------------------------');
disp('[Start] Wrapping measurements with EXO in y...');
for blockIdx = 1 : block.nrOfBlocks
    fext.rightHuman = shoes(blockIdx).Right_HF;
    fext.leftHuman  = shoes(blockIdx).Left_HF;
    
    dataWithEXOiny(blockIdx).block = block.labels(blockIdx);
    dataWithEXOiny(blockIdx).data  = dataPackaging(blockIdx, ...
        humanModel,...
        humanSensors,...
        suit_runtime,...
        fext,...
        synchroKin(blockIdx).ddq,...
        bucket.contactLink, ...
        priors);

    % y vector as input for MAP
    [dataWithEXOiny(blockIdx).y, dataWithEXOiny(blockIdx).Sigmay] = berdyMeasurementsWrapping(berdy, ...
        dataWithEXOiny(blockIdx).data);
end
disp('[End] Wrapping measurementswith EXO in y');
% ---------------------------------------------------
% CHECK: print the order of measurement in y
% printBerdySensorOrder(berdy);
% ---------------------------------------------------

%% MAP computation with EXO in y
if ~exist(fullfile(bucket.pathToProcessedData,'estimationWithEXOiny.mat'), 'file')
    for blockIdx = 1 : block.nrOfBlocks
        priors.Sigmay = dataWithEXOiny(blockIdx).Sigmay;
        estimationWithEXOiny(blockIdx).block = block.labels(blockIdx);
        if opts.Sigma_dgiveny
            disp('-------------------------------------------------------------------');
            disp(strcat('[Start] Complete MAP computation with EXO in y for Block ',num2str(blockIdx),'...'));
            [estimationWithEXOiny(blockIdx).mu_dgiveny, estimationWithEXOiny(blockIdx).Sigma_dgiveny] = MAPcomputation_floating(berdy, ...
                traversal, ...
                synchroKin(blockIdx),...
                dataWithEXOiny(blockIdx).y, ...
                priors, ...
                baseVel(blockIdx).baseAngVelocity, ...
                'SENSORS_TO_REMOVE', sensorsToBeRemoved);
            disp(strcat('[End] Complete MAP computation with EXO in y for Block ',num2str(blockIdx)));
            % TODO: variables extraction
            % Sigma_tau extraction from Sigma d --> since sigma d is very big, it
            % cannot be saved! therefore once computed it is necessary to extract data
            % related to tau and save that one!
            % TODO: extractSigmaOfEstimatedVariables
        else
            disp('-------------------------------------------------------------------');
            disp(strcat('[Start] mu_dgiveny MAP computation with EXO in y for Block ',num2str(blockIdx),'...'));
            [estimationWithEXOiny(blockIdx).mu_dgiveny] = MAPcomputation_floating(berdy, ...
                traversal, ...
                synchroKin(blockIdx),...
                dataWithEXOiny(blockIdx).y, ...
                priors, ...
                baseVel(blockIdx).baseAngVelocity, ...
                'SENSORS_TO_REMOVE', sensorsToBeRemoved);
            disp(strcat('[End] mu_dgiveny MAP computation with EXO in y for Block ',num2str(blockIdx)));
        end
    end
    save(fullfile(bucket.pathToProcessedData,'estimationWithEXOiny.mat'),'estimationWithEXOiny');
else
    load(fullfile(bucket.pathToProcessedData,'estimationWithEXOiny.mat'));
end

%% Variables extraction from MAP estimation with EXO in y
if ~exist(fullfile(bucket.pathToProcessedData,'estimatedVariablesWithEXOiny.mat'), 'file')
    % torque extraction via Berdy
    for blockIdx = 1 : block.nrOfBlocks
        disp('-------------------------------------------------------------------');
        disp(strcat('[Start] Torque extraction with EXO in y for Block ',num2str(blockIdx),'...'));
        estimatedVariablesWithEXOiny.tau(blockIdx).block  = block.labels(blockIdx);
        estimatedVariablesWithEXOiny.tau(blockIdx).label  = selectedJoints;
        estimatedVariablesWithEXOiny.tau(blockIdx).values = extractEstimatedTau_from_mu_dgiveny(berdy, ...
            estimationWithEXOiny(blockIdx).mu_dgiveny, ...
            synchroKin(blockIdx).q);
        disp(strcat('[End] Torque extraction with EXO in y for Block ',num2str(blockIdx)));
    end
    % fext extraction, manual (no Berdy)
    for blockIdx = 1 : block.nrOfBlocks
        disp('-------------------------------------------------------------------');
        disp(strcat('[Start] External force extraction with EXO in y for Block ',num2str(blockIdx),'...'));
        estimatedVariablesWithEXOiny.Fext(blockIdx).block  = block.labels(blockIdx);
        estimatedVariablesWithEXOiny.Fext(blockIdx).label  = dVectorOrder;
        estimatedVariablesWithEXOiny.Fext(blockIdx).values = extractEstimatedFext_from_mu_dgiveny(berdy, ...
            dVectorOrder, ...
            estimationWithEXOiny(blockIdx).mu_dgiveny);
        disp(strcat('[End] External force extraction with EXO in y for Block ',num2str(blockIdx)));
    end
    save(fullfile(bucket.pathToProcessedData,'estimatedVariablesWithEXOiny.mat'),'estimatedVariablesWithEXOiny');
else
    load(fullfile(bucket.pathToProcessedData,'estimatedVariablesWithEXOiny.mat'));
end

%% Simulated y with EXO in y
% This section is useful to compare the measurements in the y vector and
% the results of the MAP.  Note: you cannot compare directly the results of
% the MAP (i.e., mu_dgiveny) with the measurements in the y vector but you
% have to pass through the y_sim and only later to compare y and y_sim.
if ~exist(fullfile(bucket.pathToProcessedData,'y_simWithEXOiny.mat'), 'file')
    for blockIdx = 1 : block.nrOfBlocks
        disp('-------------------------------------------------------------------');
        disp(strcat('[Start] Simulated y computation with EXO in y for Block ',num2str(blockIdx),'...'));
        y_simWithEXOiny(blockIdx).block = block.labels(blockIdx);
        [y_simWithEXOiny(blockIdx).y_sim] = sim_y_floating(berdy, ...
            synchroKin(blockIdx),...
            traversal, ...
            baseVel(blockIdx).baseAngVelocity, ...
            estimationWithEXOiny(blockIdx).mu_dgiveny);
        disp(strcat('[End] Simulated y computation with EXO in y for Block ',num2str(blockIdx)));
    end
    save(fullfile(bucket.pathToProcessedData,'y_simWithEXOiny.mat'),'y_simWithEXOiny');
else
    load(fullfile(bucket.pathToProcessedData,'y_simWithEXOiny.mat'));
end
