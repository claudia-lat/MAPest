function [ state, ddq, selectedJoints ] = IK(filenameOsimModel, filenameTrc, setupFile)
%I K function computes the Inverse Kinematics computation by using the
% OpenSim API.  After computing q angles, it uses  Savitzi-Golay for
% obtaining dq and ddq.  Outputs: state and ddq are in radians.

%% Use OpenSim InverseKinematicTool
import org.opensim.modeling.*  % import OpenSim APIs to be used in MATLAB
osimModel = Model(filenameOsimModel);
osimModel.initSystem();
% generic experiment/model info are loaded from the file.xml:
ikTool = InverseKinematicsTool(setupFile);
% subject info are set manually:
ikTool.setModel(osimModel);
ikTool.setMarkerDataFileName(filenameTrc);
outputMotionFilename = tempname;
ikTool.setOutputMotionFileName(outputMotionFilename);
ikTool.run();

%% Extract data from motion file.mot
% motionFile = fileread('./data/subject1_bowingtask.mot');
motionData = importdata(outputMotionFilename);
delete(outputMotionFilename);

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
Sg.samplingTime = 1/240; % 240Hz is the frame rate of Xsens data.
Sg.polinomialOrder = 3;
Sg.window = 57;
%[Sg.time, ~] = angleFromName(motionData, 'time');
state.q  = zeros(size(motionData.data,2)-7, size(motionData.data,1)); 
state.dq = zeros(size(state.q));
ddq      = zeros(size(state.q));

state.q = motionData.data(:, 8:end)';  % in deg
[~,state.dq,ddq] = SgolayFilterAndDifferentiation(Sg.polinomialOrder,Sg.window,state.q,Sg.samplingTime); % in deg

%% Transformation in radians
ddq      = ddq * pi/180;      % in rad
state.q  = state.q * pi/180;  % in rad
state.dq = state.dq * pi/180; % in rad

end
