
clear;clc;close all;
rng(1); % forcing the casual generator to be const

%% Add src to the path
addpath(genpath('src')); 
addpath(genpath('../../src'));

%% Generate subject URDF model
subjectID = 1;
H = 1.70; % set manually the heigth in [m] of the subject;
M = 61;   % set manually the mass in [kg] of the subject;
subjectParams  = findAnthropometricParams(H,M);
filenameURDF = sprintf('models/XSensURDF_subj%d.urdf',subjectID);
URDFmodel = createXsensLikeURDFmodel(subjectParams,filenameURDF);

%% Load measurements from sensors
% SUIT
mvnxFilename = 'data/S_1bowingtask.mvnx';
suit = extractSuitData(mvnxFilename,'data');
% FORCEPLATE  --> to be done!

%% Generate subject OSIM model
filenameOSIM = sprintf('models/XSensOSIM_subj%d.osim',subjectID);
OSIMmodel = createXsensLikeOSIMmodel(subjectParams, suit, filenameOSIM);

%% Load URDF model
% model.FileName = sprintf('models/XSensURDF_subj%d.urdf',subjectID);
model.filename = filenameURDF;
modelLoader = iDynTree.ModelLoader();
if ~modelLoader.loadModelFromFile(model.filename);
    fprint('Something wrong with the model loading.')
end

% % LOAD MODEL FROM STRING  --> to be fixed, it doesn't work yet!!!
% modelLoader = iDynTree.ModelLoader();
% if ~modelLoader.loadModelFromString('urdfModel');
%     fprint('Something wrong with the model loading.')
% end

model   = modelLoader.model(); 
% sensors = modelLoader.sensors(); ->TO BE ADDED
                 