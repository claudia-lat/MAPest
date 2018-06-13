clc; close all; clear all;

rng(1); % Force the casual generator to be const
format long;

%% Add src to the path
addpath(genpath('src'));
addpath(genpath('templates'));
addpath(genpath('../../src'));
addpath(genpath('../../external'));

%% Java path needed by OSIM - DO NOT UNCOMMENT-SCREWS MATLAB CONFIGURATION
%  Routine left here just for Legacy, not to be used since it erases the
%  current path. 
%  setupJAVAPath();

%% Configure
% Root folder of the dataset
bucket.datasetRoot = fullfile('D:\Datasets\2018_Feb_JSI');

% Subjects and tasks to be processed
subjectList = [5];
taskList = [1];

%% Run MAPest
% Loop through all the subjects
for s = 1:length(subjectList)
    subjectID = subjectList(s);
    
    % Loop through all the tasks 
    for t = 1:length(taskList)
        taskID = taskList(t);
        
        % Clear the workspace apart for the variables required to run the
        % loops
        clearvars -except subjectList subjectID s taskList taskID t datasetRoot
        bucket = struct;
        bucket.datasetRoot = datasetRoot;
        
        % Run MAPest main
        main;        
    end % Tasks loop
end % Subjects loop
