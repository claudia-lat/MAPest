
clc;close all;clear all;
%clear;clc;close all;
rng(1); % forcing the casual generator to be const
format long;

%% Add src to the path
addpath(genpath('src'));
addpath(genpath('templates'));
addpath(genpath('../../src'));
addpath(genpath('../../external'));

%% Java path needed by OSIM
setupJAVAPath();

%% Preliminaries
% Create a structure 'bucket' where storing different stuff generating by
% running the code
bucket = struct;
subjectID = 1;
taskID = 1;

bucket.datasetRoot = fullfile(pwd, 'dataJSI');
%bucket.datasetRoot = fullfile('D:\Datasets\2018_Feb_JSI');

%% main.m calling
main;
