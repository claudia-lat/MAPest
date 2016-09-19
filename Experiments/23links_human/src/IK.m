function [ state, ddq, selectedJoints ] = IK(filenameOsimModel, filenameTrc, setupFile)
%IK Summary of this function goes here
%   Detailed explanation goes here

%% Use OpenSim InverseKinematicTool
import org.opensim.modeling.*  % import OpenSim APIs to be used in MATLAB
osimModel = Model(filenameOsimModel);
osimModel.initSystem();
% generic experiment/model info are loaded from the file.xml:
ikTool = InverseKinematicsTool(setupFile);
% subject info are set manually:
ikTool.setModel(osimModel);
ikTool.setMarkerDataFileName(fullfile(pwd,filenameTrc));
outputMotionFilename = tempname;
ikTool.setOutputMotionFileName(outputMotionFilename);
ikTool.run();

%% Extract data from motion file.mot
% motionFile = fileread('./data/subject1_bowingtask.mot');
motionData = importdata(outputMotionFilename);
delete(outputMotionFilename);

%% Create a joint name vector ordered as in OSIM
selectedJoints = iDynTree.StringVector();
jointNameTest = cell(66,1);
for i = 8 : size(motionData.colheaders,2)
     selectedJoints.push_back(motionData.colheaders{i});
     jointNameTest{i} = motionData.colheaders{i};
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
state.q  = zeros(size(motionData.data,1),size(motionData.data,2)-7);
state.dq = zeros(size(state.q));
ddq      = zeros(size(state.q));

state.q = motionData.data(:, 8:end);
for i = 1 : size((state.q),2)
    [state.dq(:,i),ddq(:,i)] = SgolayDerivation(Sg.polinomialOrder,Sg.window,state.q(:,i),Sg.samplingTime);
end



% function [q, found] = angleFromName(struct, angleName)
%     %ANGLEFROMNAME
%     % Inputs:
%     % struct     : struct motionData from OSIM computation;
%     % angleName  : string denoting the angle q you are looking for;
%     % Outputs:
%     % q      : vector of joint angles;
%     % found  : true if q has been found, false otherwise.
% 
%     for indx = 1 : size(struct.data,2)
%         if  strcmp(struct.colheaders{1,indx},angleName)
%             q = struct.data(:,indx);
%             found = true;
%             break;
%         else
%             found = false;
%         end
%     end
%     if found == false
%         error(sprintf('Wrong joint angles matching! Angleq label <%s> not found.',angleName));
%     end
% end

end
