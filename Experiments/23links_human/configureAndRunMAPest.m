clc; close all; clear all;

rng(1); % Force the casual generator to be const
format long;

%% Add src to the path
addpath(genpath('src'));
addpath(genpath('../../src'));
addpath(genpath('../../external'));

%% Set Java path needed by OSIM - SCREWS MATLAB CURRENT CONFIGURATION
%       UNCONMMENT ONLY IF YOU KNOW WHAT YOU ARE DOING
%  Routine left here just for Legacy, not to be used since it erases the
%  current path. 

setupJAVAPath();

%% Preliminaries
% Create a structure 'bucket' where storing different stuff generating by
% running the code
bucket = struct;

%% Configure
% Root folder of the dataset
bucket.datasetRoot = fullfile(pwd, 'dataJSI');
%bucket.datasetRoot = fullfile('D:\Datasets\2018_Feb_JSI');

% Subject and task to be processed
subjectID = 1;
taskID = 1;

% EXO option
opts.EXO = true;
if opts.EXO
    opts.EXO_torqueLevelAnalysis = false;
    opts.EXO_forceLevelAnalysis  = false;
    opts.EXO_insideMAP           = false;
end

% Option for C7 joints as follows:
% - fixed in the URDF model  (i.e., opts.noC7joints = true)
% - locked on the Osim model (i.e., opts.noC7joints = true)
opts.noC7joints = false;


% ID comparisons for MAP benchmarking
opts.MAPbenchmarking = false;
if opts.MAPbenchmarking
    opts.iDynID_kinDynClass = false;
    opts.iDynID_estimClass  = false;
%     opts.OsimID             = false;
end

% Plots for the standalone analysis. No comparison for the same subject
% with and without exo
opts.finalPlot_standalone = false;

%% Run MAPest main.m
main;
