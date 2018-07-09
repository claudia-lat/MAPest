function [ state, ddq, selectedJoints] = IK(filenameOsimModel, filenameTrc, setupFile, frameRate, motFilename)
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
% [Sg.time, ~] = angleFromName(motionData, 'time');
state.q  = zeros(size(motionData.data,2)-7, size(motionData.data,1));
state.dq = zeros(size(state.q));
ddq      = zeros(size(state.q));

state.q = motionData.data(:, 8:end)';  % in deg
[~,state.dq,ddq] = SgolayFilterAndDifferentiation(Sg.polinomialOrder,Sg.window,state.q,Sg.samplingTime); % in deg

% Transformation in radians
ddq      = ddq * pi/180;      % in rad
state.q  = state.q * pi/180;  % in rad
state.dq = state.dq * pi/180; % in rad

%% Create a struct for ground base pose
% % Here we consider columns from 2 to 7 (ground joints).
% for qtyIdx = 1 : 6
%     groundBasePose(qtyIdx).quantity = motionData.colheaders(:,qtyIdx+1);
%     if qtyIdx <= 3
%         % ANGLES
%         % jGroundBase_r(z/y/x) is the rotation angles provided in deg
%         values = motionData.data(:,qtyIdx+1);
%         [~,firstOrderDeriv,~] = SgolayFilterAndDifferentiation(Sg.polinomialOrder,Sg.window,values',Sg.samplingTime);
%         groundBasePose(qtyIdx).values = values' * pi/180;
%         groundBasePose(qtyIdx).firstOrderDeriv = firstOrderDeriv * pi/180;
%     else
%         % POSITIONS
%         % jGroundBase_t(x/y/z) is the translation provided in m
%         groundBasePose(qtyIdx).values   = motionData.data(:,qtyIdx+1)';
%     end
% end
end
