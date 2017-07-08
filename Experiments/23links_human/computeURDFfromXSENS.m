% This script allows to extract from a Xsens acquisition those data that
% are useful for creating the URDF model of the human subject involved in
% the experiment.
%
% Note: nothing new from the code that already exists.  It is only a
% shortcut to obtain the URDF model without using any other file/struct.
%
% The script requires:
% - to create manually in the folder 'URDFfromXSENS' a folder name 
%   'Subj_0x' where x is the progressive number of the subject
% - to have in this folder a file .mvnx named 'Subj-0%d.mvnx' exported from
%   Xsens MVN Studio
% - to set manually the same number x in 'bucket.subjectID'
% - to insert manually the mass of the subject in 'bucket.mass'
%
% Here there is an example with a subject 0 (of mass 70 Kg) and the 
% related .mvnx acquisition file.

%% General settings
clear;clc;close all;
rng(1); % forcing the casual generator to be const

% Add paths
addpath(genpath('src')); 
addpath(genpath('templates')); 
addpath(genpath('../../src'));

bucket = struct;
bucket.subjectID = 0;
bucket.saveFolder = sprintf(fullfile(pwd,'URDFfromXsens/Subj_0%d'),bucket.subjectID);

%% Load measurements from .mvnx file and create SUIT struct
if ~exist(fullfile(bucket.saveFolder,'suit.mat'))
    bucket.mvnxFilename = sprintf('URDFfromXsens/Subj_0%d/Subj-0%d.mvnx',bucket.subjectID,bucket.subjectID);
    suit = extractSuitData(bucket.mvnxFilename);
    suit = computeSuitSensorPosition(suit); % obtain sensors position
    save(fullfile(bucket.saveFolder,'/suit.mat'),'suit');
else
    load(fullfile(bucket.saveFolder,'suit.mat'));
end

%% Extract subject parameters from SUIT
% without a measurement form forceplat, this parameter (in Kg) has to be 
% manually set
bucket.mass = 70;  
subjectParamsFromData = subjectParamsComputation(suit, bucket.mass);

%% Create URDF model
bucket.filenameURDF = sprintf(fullfile(bucket.saveFolder,'/XSensURDF_subj-0%d.urdf'),bucket.subjectID);
bucket.URDFmodel = createXsensLikeURDFmodel(subjectParamsFromData, ...
                                            suit.sensors,...
                                            'filename',bucket.filenameURDF,...
                                            'GazeboModel',false);
disp('----------------');
disp(sprintf('URDF model for subject 0%d created!',bucket.subjectID));
disp('----------------');

