function [ state, ddq, selectedJoints] = IK(filenameOsimModel, filenameTrc, setupFile, frameRate, motFilename)
%IK computes the Inverse Kinematics by using the OpenSim API.
% After computing joint angles q, it exploits Savitzi-Golay for
% obtaining joint velocities and accelerations (dq, ddq).
%
% Outputs:
% - state : (q, dq) in rad
% - ddq   : in rad
% - selectedJoints: order of the joints in then Osim model

%% Use OpenSim InverseKinematicTool
import org.opensim.modeling.*  % import OpenSim APIs to be used in MATLAB
osimModel = Model(filenameOsimModel);
osimModel.initSystem();
% generic experiment/model info are loaded from the template .xml:
ikTool = InverseKinematicsTool(setupFile);
% subject info are set manually:
ikTool.setModel(osimModel);
ikTool.setMarkerDataFileName(filenameTrc);
outputMotionFilename = motFilename;
ikTool.setOutputMotionFileName(outputMotionFilename);
if ~exist(outputMotionFilename, 'file')
    ikTool.run();
end

%% Extract data from motion file.mot
motionData = importdata(outputMotionFilename);
% delete(outputMotionFilename);

%% Create a joint name vector ordered as in OSIM
selectedJoints = cell(size(motionData.colheaders,2)-7,1);
for i = 8 : size(motionData.colheaders,2)
    selectedJoints{i-7} = motionData.colheaders{i};
end
% 8 is the column from which starting to select joints. Column 1 (time) and
% columns from 2 to 7 (ground joints) will be discarded. We decide here
% what will be the order of joints because this order will be passed to the
% modelLoader in the main.

%% Savitzi-Golay computation
% set Sg parameters
Sg.samplingTime = 1/frameRate;
Sg.polinomialOrder = 3;
Sg.window = 57;

state.q  = zeros(size(motionData.data,2)-7, size(motionData.data,1));
state.dq = zeros(size(state.q));
ddq      = zeros(size(state.q));

state.q = motionData.data(:, 8:end)';  % in deg
[~,state.dq,ddq] = SgolayFilterAndDifferentiation(Sg.polinomialOrder,Sg.window,state.q,Sg.samplingTime); % in deg

% Transformation in radians
ddq      = ddq * pi/180;      % in rad
state.q  = state.q * pi/180;  % in rad
state.dq = state.dq * pi/180; % in rad

end
