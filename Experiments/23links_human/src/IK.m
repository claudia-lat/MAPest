function [ q, dq, ddq ] = IK(filenameOsimModel, filenameTrc, setupFile)
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
%ikTool.setName('subject1');
ikTool.setMarkerDataFileName(fullfile(pwd,filenameTrc));
%ikTool.setMarkerDataFileName(strcat('../',filenameTrc));
outputMotionFilename = tempname;% 'subject1_bowingtask.mot';
ikTool.setOutputMotionFileName(outputMotionFilename);
ikTool.run();
%% Extract data from motion file.mot
%motionFile = fileread('./data/subject1_bowingtask.mot');
motionData = importdata(outputMotionFilename);
delete(outputMotionFilename);
%% Savitzi-Golay computation
Sg.samplingTime = 1/240; % 240Hz is the frame rate of Xsens data.
Sg.polinomialOrder = 3;
Sg.window = 57;
[Sg.time, ~] = angleFromName(motionData, 'time');
% example for one angle
[q, ~]   = angleFromName(motionData, 'jLeftBallFoot_angle1');
[dq,ddq] = SgolayDerivation(Sg.polinomialOrder,Sg.window,q,Sg.samplingTime);
%
% ..to be continued



function [q, found] = angleFromName(struct, angleName)
    %ANGLEFROMNAME
    % Inputs:
    % struct     : struct motionData from OSIM computation;
    % angleName  : string denoting the angle q you are looking for;
    % Outputs:
    % q      : vector of joint angles;
    % found  : true if q has been found, false otherwise.

    for indx = 1 : size(struct.data,2)
        if  strcmp(struct.colheaders{1,indx},angleName)
            q = struct.data(:,indx);
            found = true;
            break;
        else
            found = false;
        end
    end
    if found == false
        error(sprintf('Wrong joint angles matching! Angleq label <%s> not found.',angleName));
    end
end

end
