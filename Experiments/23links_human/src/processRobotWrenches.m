function [robot] = processRobotWrenches(indx, robot_kinDyn, ...
                                        human_kinDyn, robot_jntPos, human_jntPos, ...
                                        robot, ...
                                        subjectParamsFromData)
% PROCESSROBOTWRENCHES processes raw external wrenches estimates coming 
% from the robot.
%
% Inputs:
% - robot_kinDyn         : iDynTree.KinDynComputations built from robot model
% - human_kinDyn         : iDynTree.KinDynComputations built from human model
% - robot_jntPos         : Matlab vector of robot joint positions 
% - human_jntPos         : Matlab vector of human joint positions 
% - robot                : robot struct from which extract left and right 
%                          arm wrench (linear-angular), as obtained
%                          from wholeBodyDynamics
% - subjectParams        : Subject parameters (to get the ankle height)
%
% Outputs:
% - robot   : updated struct containing new fields: human left hand external 
%             wrench (humanLeftHandWrench) and human right hand external 
%             wrench (humanRightHandWrench).
%
% External wrenches are estimated by the robot in a frame (origin and
% orientation) that is local to the contact link. Furthermore is tipically 
% estimating the wrench *applied* on the robot. 
% For the human estimation we need to get from this wrenches but: 
% - multiplied by -1 (as the wrench applied on the human is exactly the 
%   opposite of the one excerted on the robot)
% - express in the frame of the human link in contact. 
%
% This function computes the fwd kinematics of the robot and the human,
% and using this information and the relative information between the
% human and the robot, given by the fixture.pdf file, correctly processes 
% the robot wrenches.  
%
% Important Robot Frames:
% l_sole frame on the sole of the robot, used as a reference in fixture.pdf
% l_elbow_1, r_elbow_1 (link) frames in which the external wrenches are expressed
%
% Important human frames:
% LeftSole frame on the sole of the human, used as a reference in  fixture.pdf 
% (currently not include in URDF)
% LeftFoot link (frame) rigidly fixed to the left foot, located in the
% ankle axis. 
% LeftHand, RightHand (link) frames used  


gravityZero = iDynTree.Vector3();
gravityZero.zero();
% First set the joint positions in the human and robot model 
% At the moment we cannot set just positions, so we also add some dummy
% joint velocities 
robotZeroJntVel = iDynTree.JointDOFsDoubleArray(robot_kinDyn.model());
robotZeroJntVel.zero();
robotJntPos = iDynTree.JointPosDoubleArray(robot_kinDyn.model());
robotJntPos.fromMatlab(robot_jntPos);
robot_kinDyn.setRobotState(robotJntPos,robotZeroJntVel,gravityZero);

humanZeroJntVel = iDynTree.JointDOFsDoubleArray(human_kinDyn.model());
humanZeroJntVel.zero();
humanJntPos = iDynTree.JointPosDoubleArray(human_kinDyn.model());
humanJntPos.fromMatlab(human_jntPos);
human_kinDyn.setRobotState(humanJntPos,humanZeroJntVel,gravityZero);

% This transform can be easily extracted from fixture.pdf 
l_sole_H_LeftSoleRot = iDynTree.Rotation();
l_sole_H_LeftSoleRot.fromMatlab([-1.0, 0.0, 0.0; ...
                                  0.0,-1.0, 0.0; ...
                                  0.0, 0.0, 1.0]);
l_sole_H_LeftSolePos = iDynTree.Position();
l_sole_H_LeftSolePos.fromMatlab([0.59; -0.2549; 0.0]);
l_sole_H_LeftSole = iDynTree.Transform(l_sole_H_LeftSoleRot,l_sole_H_LeftSolePos);

% This transform could also be added in the URDF file, TODO
LeftSole_H_LeftFootRot = iDynTree.Rotation.Identity();
LeftSole_H_LeftFootPos = iDynTree.Position();
LeftSole_H_LeftFootPos.fromMatlab([0.0; 0.0; subjectParamsFromData.leftFoot_z]);
LeftSole_H_LeftFoot =  iDynTree.Transform(LeftSole_H_LeftFootRot,LeftSole_H_LeftFootPos);

% Convinient computation
l_sole_H_LeftFoot = l_sole_H_LeftSole*LeftSole_H_LeftFoot;

% Compute robot left arm ---> human right hand trasforms 
l_elbow_1_H_l_sole = robot_kinDyn.getRelativeTransform('l_elbow_1','l_sole');
LeftFoot_H_RightHand = human_kinDyn.getRelativeTransform('LeftFoot','RightHand');
l_elbow_1_H_RightHand = l_elbow_1_H_l_sole*l_sole_H_LeftFoot*LeftFoot_H_RightHand;
RightHand_H_l_elbow_1 = l_elbow_1_H_RightHand.inverse();

% Compute robot right arm ---> human left hand trasforms 
r_elbow_1_H_l_sole = robot_kinDyn.getRelativeTransform('r_elbow_1','l_sole');
LeftFoot_H_LeftHand = human_kinDyn.getRelativeTransform('LeftFoot','LeftHand');
r_elbow_1_H_LeftHand = r_elbow_1_H_l_sole*l_sole_H_LeftFoot*LeftFoot_H_LeftHand;
LeftHand_H_r_elbow_1 = r_elbow_1_H_LeftHand.inverse();

% Transform the wrench in the appropriate frame and change the sign 
robotLeftArmWrench(1:3)  = robot.data.links.leftarm.forces(:,indx);
robotLeftArmWrench(4:6)  = robot.data.links.leftarm.moments(:,indx);
robotRightArmWrench(1:3) = robot.data.links.rightarm.forces(:,indx);
robotRightArmWrench(4:6) = robot.data.links.rightarm.moments(:,indx);

robot.processedData.humanLeftHandWrench(:,indx) = -1*(LeftHand_H_r_elbow_1.asAdjointTransformWrench().toMatlab()*robotRightArmWrench');
robot.processedData.humanRightHandWrench(:,indx) = -1*(RightHand_H_l_elbow_1.asAdjointTransformWrench().toMatlab()*robotLeftArmWrench');
end
