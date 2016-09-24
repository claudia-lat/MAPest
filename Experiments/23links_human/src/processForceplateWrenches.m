function [humanLeftFootWrench, humanRightFootWrench] = processForceplateWrenches(forceplate, subjectParams)
% PROCESSFORCEPLATEWRENCHES process raw external wrenches estimates coming from the forceplate  

% Inputs:
% forceplate : for extract forceplate 1 and 2 wrenches (linear-angular);
% contactLink : links in contact with the forceplate;
% subjectParams : subject parameters (to get the ankle height);

% Outputs:
% humanRightFootWrench : Human right foot external wrench; 
% humanLeftFootWrench  : Human left foot external wrench. 

% External wrenches are estimated by the forceplate in its frame (origin and
% orientation) that is located at a known position. 
% For the human estimation we need to get from this wrenches but: 
% - multiplied by -1 (as the wrench applied on the human is exactly the 
% opposite of the one excerted on the forceplate)
% - express in the frame of the human link in contact. 
% This function computes the wrenches that the forceplate excert on the link 
% in contact thanks to the relative information between the human and the forceplate,
% given by the fixture.pdf file. 

gravityZero = iDynTree.Vector3();
gravityZero.zero();

% Transformation matrix for forceplate 1 and 2
% This transform can be easily extracted from ../fixture/fixture.pdf
calibrationForceplate1Pos = iDynTree.Position();
calibrationForceplate1Pos.fromMatlab([0.0005; -0.0005; 0.0401]);
leftSole_H_fpRot = iDynTree.Rotation();
leftSole_H_fpRot.fromMatlab([ 1.0, 0.0, 0.0; ...
                              0.0,-1.0, 0.0; ...
                              0.0, 0.0,-1.0]);
leftSole_H_fpPos = iDynTree.Position();
leftSole_H_fpPos.fromMatlab([0.108; 0.107; 0]);
leftFoot_H_LeftSolePos = iDynTree.Position();
leftFoot_H_LeftSolePos.fromMatlab([0.0; 0.0; -subjectParams.leftFootBoxOrigin(3)]);
leftFoot_H_fp = iDynTree.Transform(leftSole_H_fpRot,leftFoot_H_LeftSolePos+leftSole_H_fpPos+calibrationForceplate1Pos);

calibrationForceplate2Pos = iDynTree.Position();
calibrationForceplate2Pos.fromMatlab([0.0002; 0.0006; 0.0398]);
rightSole_H_fpRot = iDynTree.Rotation();
rightSole_H_fpRot.fromMatlab([-1.0, 0.0, 0.0; ...
                               0.0, 1.0, 0.0; ...
                               0.0, 0.0,-1.0]);
rightSole_H_fpPos = iDynTree.Position();
rightSole_H_fpPos.fromMatlab([0.108; -0.107; 0]);
rightFoot_H_RightSolePos = iDynTree.Position();
rightFoot_H_RightSolePos.fromMatlab([0.0; 0.0; -subjectParams.rightFootBoxOrigin(3)]);
rightFoot_H_fp = iDynTree.Transform(rightSole_H_fpRot,rightFoot_H_RightSolePos+rightSole_H_fpPos+calibrationForceplate2Pos);

% Extract wrenches from forceplate DATA
forceplate1Wrench(1:3,:) = forceplate.data.plateforms.plateform1.forces(1:3,:);
forceplate1Wrench(4:6,:) = forceplate.data.plateforms.plateform1.moments(1:3,:);
forceplate2Wrench(1:3,:) = forceplate.data.plateforms.plateform1.forces(1:3,:);
forceplate2Wrench(4:6,:) = forceplate.data.plateforms.plateform1.moments(1:3,:);

%% Transform the wrench in the appropraite frame and change the sign 
humanLeftFootWrench = -1*(leftFoot_H_fp.asAdjointTransformWrench().toMatlab()*forceplate1Wrench);
humanRightFootWrench = -1*(rightFoot_H_fp.asAdjointTransformWrench().toMatlab()*forceplate2Wrench);
