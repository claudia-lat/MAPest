
% -------------------------------------------------------------------------
% extractCSV_kinematics
% -------------------------------------------------------------------------
% Script to extract the kinematics into a CSV file. This is useful
% in order to save/share the .csv file instead of the .mat file.
%
% This script is particurly tailored for the code written into the main.m:
% https://github.com/claudia-lat/MAPest/blob/master/Experiments/23links_human/main.m

%% Preliminaries
% Check the directory for the CSV files
bucket.pathCSVfiles = fullfile(bucket.datasetRoot,'CSVfile');
if ~exist(bucket.pathCSVfiles,'dir')
    mkdir(bucket.pathCSVfiles);
end

% Go throughout your dataset
for subjIdx = 1 : length(subjectID)
    bucket.pathToSubject = fullfile(bucket.datasetRoot, sprintf('S%02d',subjectID(subjIdx)));
    
    if ~exist(fullfile(bucket.pathCSVfiles,sprintf('S%02d',subjectID(subjIdx))),'dir')
        mkdir(fullfile(bucket.pathCSVfiles,sprintf('S%02d',subjectID(subjIdx))));
    end
    
    bucket.pathToSubjectFolder = fullfile(bucket.pathCSVfiles,sprintf('S%02d',subjectID(subjIdx)));
    for taskIdx = 1 : length(taskID)
        bucket.pathToTask = fullfile(bucket.pathToSubject,sprintf('task%d',taskID(taskIdx)));
        bucket.pathToProcessedData = fullfile(bucket.pathToTask,'processed');
        
        if ~exist(fullfile(bucket.pathToSubjectFolder,sprintf('Task%d',taskID(taskIdx))),'dir')
            mkdir(fullfile(bucket.pathToSubjectFolder,sprintf('Task%d',taskID(taskIdx))));
        end
        
        bucket.pathToTaskFolder = fullfile(bucket.pathToSubjectFolder,sprintf('Task%d',taskID(taskIdx)));
        
        %% CSV conversion of the joint kinematics
        
        % Get labels of the human model joints
        load(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'));
        % Get the kinematics data, e.g., in this case the kinematics is in
        % the Matlab struct synchroKin.mat structured a sfollows:
        % synchroKin = []
        % synchroKin.timestamp
        % synchroKin.state.q
        % synchroKin.state.dq
        % synchroKin.ddq
        load(fullfile(bucket.pathToProcessedData,'synchroKin.mat'));
        
        for i = 1 : size(selectedJoints,1)
            selectedJointsList_q{i,1}   = strcat(selectedJoints{i},'_q');
            selectedJointsList_dq{i,1}  = strcat(selectedJoints{i},'_dq');
            selectedJointsList_ddq{i,1} = strcat(selectedJoints{i},'_ddq');
        end
        
        if ~exist(fullfile(bucket.pathToTaskFolder,sprintf('jointKin_S%02d_Task%d.csv',subjectID(subjIdx),taskID(taskIdx))), 'file')
            jointKin = struct;
            % Concatenate labels
            jointKinLabel  = [selectedJointsList_q', selectedJointsList_dq', selectedJointsList_ddq'];
            % Concatenate values
            jointKinValues       = [synchroKin.state.q', synchroKin.state.dq', synchroKin.ddq'];
            for labelIdx = 1 : size(jointKinLabel,2)
                for sampleIdx = 1 : size(synchroKin(blockIdx).masterTime,2)
                    jointKin(sampleIdx).masterTime = synchroKin.timestamp(sampleIdx);
                    jointKin(sampleIdx).(jointKinLabel{labelIdx}) = jointKinValues(sampleIdx,labelIdx);
                end
            end
            tmp_csvName = sprintf('jointKin_S%02d_Task%d.csv',subjectID(subjIdx),taskID(taskIdx));
            writetable(struct2table(jointKin), tmp_csvName);
            copyfile(tmp_csvName,bucket.pathToTaskFolder)
            delete(tmp_csvName);
        end
    end
end
