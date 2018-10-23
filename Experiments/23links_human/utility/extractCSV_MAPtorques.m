
% -------------------------------------------------------------------------
% extractCSV_MAPtorques
% -------------------------------------------------------------------------
% Script to extract estimated joint torques into a CSV file. This is useful
% in order to save/share the .csv file instead of the .mat file.
%
% This script is particurly tailored for the code written into the main.m:
% https://github.com/claudia-lat/MAPest/blob/master/Experiments/23links_human/main.m
% and it shoul be run after computing the MAP estimation for your dataset.

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
        
        %% CSV conversion of the estimated torques
        
        % Get the estimated values from MAP
        load(fullfile(bucket.pathToProcessedData,'estimatedVariables.mat'));
        % Get labels associated to the torques
        load(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'));
        % Get the timestamp.  NOTE: you can load here your own file where
        % there is the timestamp (e.g., in this case this info is embedded
        % in the Matlab struct synchroKin.mat).
        load(fullfile(bucket.pathToProcessedData,'synchroKin.mat'));
        
        if ~exist(fullfile(bucket.pathToTaskFolder,sprintf('jointTorque_S%02d_Task%d.csv',subjectID(subjIdx),taskID(taskIdx))), 'file')
            estimatedTorque = struct;
            for labelIdx = 1 : size(selectedJoints,1)
                for sampleIdx = 1 :size(estimatedVariables.tau.values,2)
                    estimatedTorque(sampleIdx).masterTime = synchroKin.timestamp(sampleIdx);
                    estimatedTorque(sampleIdx).(selectedJoints{labelIdx}) = estimatedVariables.tau.values(labelIdx, sampleIdx);
                    estimatedTorque(sampleIdx).(selectedJoints{labelIdx}) = estimatedVariables.tau.values(labelIdx, sampleIdx);
                end
            end
            tmp_csvName = sprintf('jointTorque_S%02d_Task%d.csv',subjectID(subjIdx),taskID(taskIdx));
            writetable(struct2table(estimatedTorque), tmp_csvName);
            copyfile(tmp_csvName,bucket.pathToTaskFolder)
            delete(tmp_csvName);
        end
    end
end
