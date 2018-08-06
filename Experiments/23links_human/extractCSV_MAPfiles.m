
%% CSV file conversion
% This script has to be launched after the overall analasys for all the
% dataset subjects!

pathCSVfiles = fullfile(bucket.datasetRoot,'CSVfile');
if ~exist(pathCSVfiles,'dir')
    mkdir(pathCSVfiles);
end

% Group options
group1 = true;
group2 = false;

% Models option
opts.noC7joints = true;

if group1
    % GROUP 1
    subjectID = [1,3,5,7,9,11];
    taskID = [0, 1];
end

if group2
    % GROUP 2
    subjectID = [2,4,6,8,10,12];
    taskID = [0,1,2];
end

% Blocks
block.labels = {'block1'; ...
    'block2'; ...
    'block3'; ...
    'block4'; ...
    'block5'};
block.nrOfBlocks = size(block.labels,1);

for subjIdx = 1 : length(subjectID)
    pathToSubject = fullfile(bucket.datasetRoot, sprintf('S%02d',subjectID(subjIdx)));
    
    if ~exist(fullfile(pathCSVfiles,sprintf('S%02d',subjectID(subjIdx))),'dir')
        mkdir(fullfile(pathCSVfiles,sprintf('S%02d',subjectID(subjIdx))));
    end
    
    pathToSubjectFolder = fullfile(pathCSVfiles,sprintf('S%02d',subjectID(subjIdx)));
    for taskIdx = 1 : length(taskID)

        % exo options
        if (group1 && taskID(taskIdx) == 0) || (group2 && taskID(taskIdx) == 1)
            EXOopts = true;
        end

        pathToTask = fullfile(pathToSubject,sprintf('task%d',taskID(taskIdx)));
        pathToProcessedData = fullfile(pathToTask,'processed');
        
        if ~exist(fullfile(pathToSubjectFolder,sprintf('Task%d',taskID(taskIdx))),'dir')
            mkdir(fullfile(pathToSubjectFolder,sprintf('Task%d',taskID(taskIdx))));
        end
        pathToTaskFolder = fullfile(pathToSubjectFolder,sprintf('Task%d',taskID(taskIdx)));
        
        %% Copy URDF model (with and without EXO)
        if opts.noC7joints
            filenameURDF = fullfile(pathToSubject, sprintf('XSensURDF_subj%02d_48dof_noC7.urdf', subjectID(subjIdx)));
            copyfile(filenameURDF,pathToSubjectFolder);
            filenameURDFexo = fullfile(pathToSubject, sprintf('XSensURDF_subj%02d_48dof_EXO_noC7.urdf', subjectID(subjIdx)));
            copyfile(filenameURDFexo,pathToSubjectFolder);
        else
            filenameURDF = fullfile(pathToSubject, sprintf('XSensURDF_subj%02d_48dof.urdf', subjectID(subjIdx)));
            copyfile(filenameURDF,pathToSubjectFolder);
            filenameURDFexo = fullfile(pathToSubject, sprintf('XSensURDF_subj%02d_48dof_EXO.urdf', subjectID(subjIdx)));
            copyfile(filenameURDFexo,pathToSubjectFolder);
        end

        %% CSV conversion of the joint kinematics
        load(fullfile(pathToProcessedData,'selectedJoints.mat'));
        load(fullfile(pathToProcessedData,'synchroKin.mat'));
        
        for i = 1 : size(selectedJoints,1)
            selectedJointsList_q{i,1}   = strcat(selectedJoints{i},'_q');
            selectedJointsList_dq{i,1}  = strcat(selectedJoints{i},'_dq');
            selectedJointsList_ddq{i,1} = strcat(selectedJoints{i},'_ddq');
        end

        if ~exist(fullfile(pathToTaskFolder,sprintf('jointKin_S%02d_Task%d_Block1.csv',subjectID(subjIdx),taskID(taskIdx))), 'file')
            jointKin = struct;
            % Concatenate labels
            jointKinLabel  = [selectedJointsList_q', selectedJointsList_dq', selectedJointsList_ddq'];
            for blockIdx = 1 : block.nrOfBlocks
                % Concatenate values
                jointKinValues       = [synchroKin(blockIdx).q', synchroKin(blockIdx).dq', synchroKin(blockIdx).ddq'];
                for labelIdx = 1 : size(jointKinLabel,2)
                    for sampleIdx = 1 : size(synchroKin(blockIdx).masterTime,2)
                        jointKin(sampleIdx).masterTime = synchroKin(blockIdx).masterTime(sampleIdx);
                        jointKin(sampleIdx).(jointKinLabel{labelIdx}) = jointKinValues(sampleIdx,labelIdx);
                    end
                end
                tmp_csvName = sprintf('jointKin_S%02d_Task%d_Block%d.csv',subjectID(subjIdx),taskID(taskIdx), blockIdx);
                writetable(struct2table(jointKin), tmp_csvName);
                copyfile(tmp_csvName,pathToTaskFolder)
                delete(tmp_csvName);
            end
        end

        %% CSV conversion of the estimated torques
        load(fullfile(pathToProcessedData,'estimatedVariables.mat'));

        % Estimation of the torques for ALL THE TASKS without considering
        % the contribution of the EXO
        if ~exist(fullfile(pathToTaskFolder,sprintf('estimatedTorque_S%02d_Task%d_Block1.csv',subjectID(subjIdx),taskID(taskIdx))), 'file')
            estimatedTorque = struct;
            for blockIdx = 1 : block.nrOfBlocks
                for labelIdx = 1 : size(selectedJoints,1)
                    for sampleIdx = 1 :size(estimatedVariables.tau(blockIdx).values,2)
                        estimatedTorque(sampleIdx).masterTime = synchroKin(blockIdx).masterTime(sampleIdx);
                        estimatedTorque(sampleIdx).(selectedJoints{labelIdx}) = estimatedVariables.tau(blockIdx).values(labelIdx, sampleIdx);
                    end
                end
                tmp_csvName = sprintf('estimatedTorque_S%02d_Task%d_Block%d.csv',subjectID(subjIdx),taskID(taskIdx), blockIdx);
                writetable(struct2table(estimatedTorque), tmp_csvName);
                copyfile(tmp_csvName,pathToTaskFolder)
                delete(tmp_csvName);
            end
        end

        % Estimation of the shoulders torques for the EXO TASKS by considering
        % the contribution of the EXO itself
        if EXOopts
            load(fullfile(pathToProcessedData,'exo.mat'))
            if ~exist(fullfile(pathToTaskFolder,sprintf('estimatedTorqueEXO_S%02d_Task%d_Block1.csv',subjectID(subjIdx),taskID(taskIdx))), 'file')
                estimatedTorqueEXO = struct;
                for blockIdx = 1 : block.nrOfBlocks
                    for sampleIdx = 1 :size(exo(blockIdx).masterTime,2)
                        estimatedTorqueEXO(sampleIdx).masterTime  = exo(blockIdx).masterTime(sampleIdx);
                        % --------- RightShoulder data (after change coordinates)
                        % angle qFirst --> it is not the q contained in synchroKin for rotx!!
                        estimatedTorqueEXO(sampleIdx).Rsho_qFirst = exo(blockIdx).Rsho_qFirst(1,sampleIdx);
                        % torque tauFirst --> it is not the MAPest tau contained in estimatedVariables.tau fro rotx!!
                        estimatedTorqueEXO(sampleIdx).Rsho_tauFirst = exo(blockIdx).Rsho_tauFirst(1,sampleIdx);
                        % Exo torque extracted from table
                        estimatedTorqueEXO(sampleIdx).Rsho_tauFromTable = exo(blockIdx).torqueFromTable_right(sampleIdx);
                        % Diff (tauFirst_MAPest - tau_EXOfromTable)
                        estimatedTorqueEXO(sampleIdx).Rsho_torqueDiff = exo(blockIdx).torqueDiff_right(sampleIdx);

                        % --------- LeftShoulder data (after change coordinates)
                        % angle qFirst --> it is not the q contained in synchroKin for rotx!!
                        estimatedTorqueEXO(sampleIdx).Lsho_qFirst = exo(blockIdx).Lsho_qFirst(1,sampleIdx);
                        % torque tauFirst --> it is not the MAPest tau contained in estimatedVariables.taufor rotx!!
                        estimatedTorqueEXO(sampleIdx).Lsho_tauFirst = exo(blockIdx).Lsho_tauFirst(1,sampleIdx);
                        % Exo torque extracted from table
                        estimatedTorqueEXO(sampleIdx).Lsho_tauFromTable = exo(blockIdx).torqueFromTable_left(sampleIdx);
                        % Diff (tauFirst_MAPest - tau_EXOfromTable)
                        estimatedTorqueEXO(sampleIdx).Lsho_torqueDiff = exo(blockIdx).torqueDiff_left(sampleIdx);
                    end
                    tmp_csvName = sprintf('estimatedTorqueEXO_S%02d_Task%d_Block%d.csv',subjectID(subjIdx),taskID(taskIdx), blockIdx);
                    writetable(struct2table(estimatedTorqueEXO), tmp_csvName);
                    copyfile(tmp_csvName,pathToTaskFolder)
                    delete(tmp_csvName);
                end
            end
        end
        % Reset exo option
        EXOopts = false;
    end
end
