function [humanForcesProvider_testData] = testModule_humanForcesProvider(selectedSample,forceplate,robot,human_state)
%testModule_humanForcesProvider provides data
%useful for the comparison with the corresponding data obtained in
%the YARP module human-forces-provider [see 
%https://github.com/robotology-playground/human-dynamics-estimation/tree/master/human-forces-provider]
%
% This function provides:
% - the input data to the YARP module (forceplate, robot, human state);
% - the output forces (from forceplates and robot) already transformed to
%   the human frames.
%
% The idea is to create with these data a dataset for testing the c++
% module by using as its input the very same input of the Matlab code and
% by comparing the module outputs with the Matlab ones.

%% Initialize a struct for all data 
humanForcesProvider_testData = struct;

%% Fill the struct with the inputs

% FORCEPLATE 1
humanForcesProvider_testData.input.forceplate.forceplate1.forces  = forceplate.data.plateforms.plateform1.forces(:,selectedSample);
humanForcesProvider_testData.input.forceplate.forceplate1.moments = forceplate.data.plateforms.plateform1.moments(:,selectedSample);
% 
% log.info_FP1 = fileread('log_templates/forceplate/info.log');
% log.info_FP1 = strrep(log.info_FP1,'port','/amti/first/analog:o');
% log.data_FP1 = fileread('log_templates/forceplate/data.log');
% log.data_FP1 = strrep(log.data_FP1, 'F', num2str((humanForcesProvider_testData.input.forceplate.forceplate1.forces)'));
% log.data_FP1 = strrep(log.data_FP1, 'M', num2str((humanForcesProvider_testData.input.forceplate.forceplate1.moments)'));

% FORCEPLATE 2
humanForcesProvider_testData.input.forceplate.forceplate2.forces  = forceplate.data.plateforms.plateform2.forces(:,selectedSample);
humanForcesProvider_testData.input.forceplate.forceplate2.moments = forceplate.data.plateforms.plateform2.moments(:,selectedSample);

% HUMAN CONFIGURATION
% human q and dq are provided by OpenSim IK computation with the OpenSim
% order.  This order was the same of the selectedJoints order as it is 
% passed to the berdy modelLoader.  
% Human configuration comes in deg from the internal OpenSim computation and
% transformed in rad by the function IK.m.  
humanForcesProvider_testData.input.humanConfiguration.human_q  = human_state.q(:,selectedSample);  %in rad
humanForcesProvider_testData.input.humanConfiguration.human_dq = human_state.dq(:,selectedSample); %in rad/s

% ROBOT RIGHT ARM WRENCH
humanForcesProvider_testData.input.robot.rightArm.forces  = robot.data.links.rightarm.forces(:,selectedSample);
humanForcesProvider_testData.input.robot.rightArm.moments = robot.data.links.rightarm.moments(:,selectedSample);

% ROBOT LEFT ARM WRENCH
humanForcesProvider_testData.input.robot.leftArm.forces  = robot.data.links.leftarm.forces(:,selectedSample);
humanForcesProvider_testData.input.robot.leftArm.moments = robot.data.links.leftarm.moments(:,selectedSample);

% ROBOT CONFIGURATION
% for YARP testing you need the configuration of the robot in deg.  In this
% code, the struct robot contains the robot configuration in rad so we need
% to transform it in deg.
humanForcesProvider_testData.input.robotConfiguration.rightArm = 180/pi .* robot.data.q.rightArm(:,selectedSample); %right arm state (16x)
humanForcesProvider_testData.input.robotConfiguration.leftArm  = 180/pi .* robot.data.q.leftArm(:,selectedSample);  %left arm state (16x)
humanForcesProvider_testData.input.robotConfiguration.rightLeg = 180/pi .* robot.data.q.rightLeg(:,selectedSample); %right leg state (6x)
humanForcesProvider_testData.input.robotConfiguration.leftLeg  = 180/pi .* robot.data.q.leftLeg(:,selectedSample);  %left leg state (6x)
humanForcesProvider_testData.input.robotConfiguration.torso    = 180/pi .* robot.data.q.torso(:,selectedSample);    %torso state (3x)

%% Fill the struct with the outputs

% FORCEPLATE 1 IN HUMAN RIGHT FOOT FRAME
humanForcesProvider_testData.output.forceplate.forceplate1 = forceplate.processedData.humanRightFootWrench(:,selectedSample);

% FORCEPLATE 2 IN HUMAN LEFT FOOT FRAME
humanForcesProvider_testData.output.forceplate.forceplate2 = forceplate.processedData.humanLeftFootWrench(:,selectedSample);

% ROBOT RIGHT ARM WRENCH IN HUMAN LEFT HAND FRAME
humanForcesProvider_testData.output.robot.rigthArm = robot.processedData.humanLeftHandWrench(:,selectedSample);

% ROBOT LEFT ARM WRENCH IN HUMAN RIGHT HAND FRAME
humanForcesProvider_testData.output.robot.leftArm = robot.processedData.humanRightHandWrench(:,selectedSample);

end
