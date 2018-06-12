
%% Kinematics info extraction
if ~exist(fullfile(bucket.pathToProcessedData,'synchroKin.mat'), 'file')
    fieldsToBeRemoved = {'RightShoe_SF','LeftShoe_SF','FP_SF'};
    synchroKin = rmfield(synchroData,fieldsToBeRemoved);
    save(fullfile(bucket.pathToProcessedData,'synchroKin.mat'),'synchroKin');
end

pathToKinFolder = fullfile(bucket.datasetRoot,'kinInfo');
if ~exist(pathToKinFolder,'dir')
    mkdir(pathToKinFolder);
end

% % Get a list of all files and folders in this folder.
% files = dir(bucket.datasetRoot);
% % Get a logical vector that tells which is a directory.
% dirFlags = [files.isdir];
% dirFlags = [files.isdir] & ~strcmp({files.name},'.') & ~strcmp({files.name},'..');
% % Extract only those that are directories.
% subFolders = files(dirFlags);
% % Print folder names to command window.
% for k = 1 : length(subFolders)
% 	fprintf('Sub folder #%d = %s\n', k, subFolders(k).name);
% end

group1 = true;
if group1
    % GROUP 1
    subjectID = [1,3,5,9,11];
    taskID = [1];
else
    % GROUP 2
    subjectID = [2,4,6,8,10];
    taskID = [0,2];
end

for subjIdx = 1 : length(subjectID)
    if ~exist(fullfile(pathToKinFolder,sprintf('S%02d',subjectID(subjIdx))),'dir')
        mkdir(fullfile(pathToKinFolder,sprintf('S%02d',subjectID(subjIdx))));
    end
    pathToSubjectFolder = fullfile(pathToKinFolder,sprintf('S%02d',subjectID(subjIdx)));
    for taskIdx = 1 : length(taskID)
        if ~exist(fullfile(pathToSubjectFolder,sprintf('Task%d',taskID(taskIdx))),'dir')
            mkdir(fullfile(pathToSubjectFolder,sprintf('Task%d',taskID(taskIdx))));
        end
        pathToTaskFolder = fullfile(pathToSubjectFolder,sprintf('Task%d',taskID(taskIdx)));
    end
    % Copy URDF model
    copyfile(bucket.filenameURDF,pathToSubjectFolder);
    % Copy selectedJoints
    copyfile(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'),pathToTaskFolder);
    % Copy groundBasePose
    copyfile(fullfile(bucket.pathToProcessedData,'groundBasePose.mat'),pathToTaskFolder);
    % Copy the kinematics
    copyfile(fullfile(bucket.pathToProcessedData,'synchroKin.mat'),pathToTaskFolder);
end
