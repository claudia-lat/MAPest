function [forceplate] = processForceplateWrenches(forceplate, subjectParamsFromData)
% PROCESSFORCEPLATEWRENCHES processes raw external wrenches estimates coming 
% from the forceplate. 
%
% Inputs:
% - forceplate            : forceplate 1 and 2 wrenches (linear-angular);
% - contactLink           : human links in contact with the forceplate;
% - subjectParamsFromData : for getting the ankle heights, i.e the origin 
%                           position of the reference frame of both feet
%                           wrt their projection on the ground silhoutte 
%                           (provided in fixture.pdf ).
% Outputs:
% - forceplate  : updated struct containing new fields : human right foot 
%                 external wrench (humanRightFootWrench) and human left 
%                 foot external wrench (humanLeftFootWrench). 
%
% External wrenches are estimated by the forceplate in its frame (origin and
% orientation) that is located at a known position. 
% For the human estimation we need to get from this wrenches but: 
% - multiplied by -1 (as the wrench applied on the human is exactly the 
%   opposite of the one excerted on the forceplate)
% - express in the frame of the human link in contact.
%
% This function computes the wrenches that the forceplate exert on the link 
% in contact thank to the relative information between the human and the 
% forceplate, given by the fixture.pdf file. 


gravityZero = iDynTree.Vector3();
gravityZero.zero();
% Transformation matrix T for forceplate 1 and 2 (this transform can be 
% easily extracted from fixture.pdf ).
calibrationForceplate1Pos = iDynTree.Position();
calibrationForceplate1Pos.fromMatlab([0.0002; 0.0006; 0.0398]);
rightSole_T_fpRot = iDynTree.Rotation();
rightSole_T_fpRot.fromMatlab([ -1.0,  0.0,  0.0; ...
                                0.0,  1.0,  0.0; ...
                                0.0,  0.0, -1.0]);
rigthSole_T_fpPos = iDynTree.Position();
rigthSole_T_fpPos.fromMatlab([0.108; -0.107; 0]);
rightFoot_T_rightSolePos = iDynTree.Position();
rightFoot_T_rightSolePos.fromMatlab([0.0; 0.0; -subjectParamsFromData.rightFootBoxOrigin(3)] );
rightFoot_T_fp = iDynTree.Transform(rightSole_T_fpRot,...
                rightFoot_T_rightSolePos + rigthSole_T_fpPos + calibrationForceplate1Pos);

            
            
calibrationForceplate2Pos = iDynTree.Position();
calibrationForceplate2Pos.fromMatlab([0.0005; -0.0005; 0.0401]);
leftSole_T_fpRot = iDynTree.Rotation();
leftSole_T_fpRot.fromMatlab([  1.0,  0.0,  0.0; ...
                               0.0, -1.0,  0.0; ...
                               0.0,  0.0, -1.0]);
leftSole_T_fpPos = iDynTree.Position();
leftSole_T_fpPos.fromMatlab([0.108; 0.107; 0]);
leftFoot_T_leftSolePos = iDynTree.Position();
leftFoot_T_leftSolePos.fromMatlab([0.0; 0.0; -subjectParamsFromData.leftFoot_z]);
leftFoot_T_fp = iDynTree.Transform(leftSole_T_fpRot,...
                 leftFoot_T_leftSolePos+leftSole_T_fpPos+calibrationForceplate2Pos);

% Extract wrenches from forceplate data 
forceplate1Wrench(1:3,:) = forceplate.data.plateforms.plateform1.forces;
forceplate1Wrench(4:6,:) = forceplate.data.plateforms.plateform1.moments;
forceplate2Wrench(1:3,:) = forceplate.data.plateforms.plateform2.forces;
forceplate2Wrench(4:6,:) = forceplate.data.plateforms.plateform2.moments;

%% Transform the wrench in the appropriate frame and change the sign 
forceplate.processedData.humanLeftFootWrench = -1*(leftFoot_T_fp.asAdjointTransformWrench().toMatlab()*forceplate2Wrench);
forceplate.processedData.humanRightFootWrench = -1*(rightFoot_T_fp.asAdjointTransformWrench().toMatlab()*forceplate1Wrench);
end
