function [robotJointPos, robotModel] = createRobotModel(robot) 

% Extract the joint position of interest from the robot struct 
robotJointPos(1,:) = robot.data.q.torso(3,:);
robotJointPos(2,:) = robot.data.q.torso(2,:);
robotJointPos(3,:) = robot.data.q.torso(1,:);
robotJointPos(4:7,:) = robot.data.q.leftArm(1:4,:);
robotJointPos(8:11,:) = robot.data.q.rightArm(1:4,:);
robotJointPos(12:17,:) = robot.data.q.leftLeg;
robotJointPos(18:23,:) = robot.data.q.rightLeg;

selectedRobotJoints = iDynTree.StringVector();     
selectedRobotJoints.push_back('torso_pitch');     
selectedRobotJoints.push_back('torso_roll');     
selectedRobotJoints.push_back('torso_yaw');    
selectedRobotJoints.push_back('l_shoulder_pitch');  
selectedRobotJoints.push_back('l_shoulder_roll');     
selectedRobotJoints.push_back('l_shoulder_yaw');     
selectedRobotJoints.push_back('l_elbow');    
selectedRobotJoints.push_back('r_shoulder_pitch');     
selectedRobotJoints.push_back('r_shoulder_roll');     
selectedRobotJoints.push_back('r_shoulder_yaw');     
selectedRobotJoints.push_back('r_elbow');     
selectedRobotJoints.push_back('l_hip_pitch');     
selectedRobotJoints.push_back('l_hip_roll');     
selectedRobotJoints.push_back('l_hip_yaw');    
selectedRobotJoints.push_back('l_knee');     
selectedRobotJoints.push_back('l_ankle_pitch');     
selectedRobotJoints.push_back('l_ankle_roll');     
selectedRobotJoints.push_back('r_hip_pitch');     
selectedRobotJoints.push_back('r_hip_roll');     
selectedRobotJoints.push_back('r_hip_yaw');     
selectedRobotJoints.push_back('r_knee');     
selectedRobotJoints.push_back('r_ankle_pitch');     
selectedRobotJoints.push_back('r_ankle_roll');

fileNameTest = 'data/iCubGenova02.urdf';
robotModel.filename = fullfile(pwd, fileNameTest);
robotModelLoader = iDynTree.ModelLoader();
if ~robotModelLoader.loadReducedModelFromFile(robotModel.filename, selectedRobotJoints)
    fprintf('Something wrong with the robot model loading.')
end
robotModel = robotModelLoader.model().copy();
end

