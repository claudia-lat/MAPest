
clear;clc;close all;
rng(1); % forcing the casual generator to be const

%% Add src to the path
addpath(genpath('src')); 
addpath(genpath('templates')); 
addpath(genpath('../../src'));

%% Java path needed by OSIM
JAVA_OSIM_PATH = '/usr/local/Cellar/opensim-core/4.0/share/doc/OpenSim/Java';
JAVA_OSIM_LIB_FOLDER = '/usr/local/Cellar/opensim-core/4.0/lib';
%get current class path
javacpath = javaclasspath('-dynamic');
found = false;
for i = 1:size(javacpath)
    if (strcmp(javacpath{i}, JAVA_OSIM_PATH))
        found = true;
        break;
    end
end
if ~found
    javaaddpath(JAVA_OSIM_PATH);
end
javapathFile = fullfile(prefdir, 'javalibrarypath.txt');
found = false;
if exist(javapathFile, 'file')
    %open and check file
    fid = fopen(javapathFile,'r');
    %for each line, trim and compare with JAVA_OSIM_LIB_FOLDER
    tline = fgetl(fid);
    while ischar(tline)
        if strcmp(strtrim(tline), JAVA_OSIM_LIB_FOLDER)
            found = true;
            break;
        end
        tline = fgetl(fid);
    end
    fclose(fid);
end
if ~found
    fid = fopen(javapathFile,'a');
    fprintf(fid, strcat(JAVA_OSIM_LIB_FOLDER, '\n'));
    fclose(fid);
end

%% Load measurements from sensors
% SUIT
mvnxFilename = 'data/S_1bowingtask.mvnx';
suit = extractSuitData(mvnxFilename,'data');
% FORCEPLATE  --> to be done!

%% Obtain sensor position suit = computeSuitSensorPosition(suit);
suit = computeSuitSensorPosition(suit);

%% Extract subject parameters from suit data 
subjectID = 1;
M = 60;
subjectParamsFromData = subjectParamsComputation(suit, M);

%% Create URDF model
filenameURDF = sprintf('models/XSensURDF_subj%d.urdf',subjectID);
URDFmodel = createXsensLikeURDFmodel(subjectParamsFromData, suit.sensors,'filename',filenameURDF,'GazeboModel',false);

%% Generate subject OSIM model
filenameOSIM = sprintf('models/XSensOSIM_subj%d.osim',subjectID);
OSIMmodel = createXsensLikeOSIMmodel(subjectParamsFromData, filenameOSIM);

%% Inverse Kinematic computation 
setupFile = ('data/fileSetup.xml');
trcFile = ('data/S_1bowingtaskForIK.trc');
[state, ~, selectedJoints] = IK(filenameOSIM, trcFile, setupFile);

%% Load URDF model
model.filename = filenameURDF;
modelLoader = iDynTree.ModelLoader();
if ~modelLoader.loadReducedModelFromFile(model.filename, selectedJoints);
    fprint('Something wrong with the model loading.')
end
%--------------------------------------------------------------------------
% IMPORTANT NOTE:
% ---------------
% Until this point the base link was 'Pelvis' but from now on  we decided
% to change it with the LeftFoot as is really fixed during the experiment.
%--------------------------------------------------------------------------
% initialize berdy
model   = modelLoader.model();
sensors = modelLoader.sensors();
berdyOptions = iDynTree.BerdyOptions;
berdyOptions.baseLink = 'LeftFoot';
% load berdy
berdy = iDynTree.BerdyHelper;
berdy.init(model, sensors, berdyOptions);
% get the current traversal
traversal = berdy.dynamicTraversal;
currentBase = berdy.model().getLinkName(traversal.getBaseLink().getIndex());
disp(strcat('Current base is: ', currentBase));
% get how the tree is visited
linkNames = cell(traversal.getNrOfVisitedLinks(), 1); %for each link in the traversal get the name
for i = 0 : traversal.getNrOfVisitedLinks() - 1
    link = traversal.getLink(i);
    linkNames{i + 1} = berdy.model().getLinkName(link.getIndex());
end


