function [forceplates] = transformForceplatesWrenches (forceplates, ...
                                                       subjectParamsFromData, ...
                                                       filenameTrc)  
% TRANSFORMFORCEPLATEWRENCHES transforms external wrenches estimated from 
% forceplates into human frames
% Inputs:
% - forceplates           : forceplate 1 and 2 wrenches in Cortex frames;
% - subjectParamsFromData : for getting the ankle heights, i.e the origin 
%                           position of the reference frame of both feet
%                           wrt their projection on the ground silhoutte 
%                           (provided in fixtureUW.pdf ).
% Outputs:
% - forceplates  : updated struct containing new fields : human right foot 
%                  external wrench (humanRightFootWrench) and human left 
%                  foot external wrench (humanLeftFootWrench). 
%
% External wrenches are estimated by the forceplates in the frame of the 
% Cortex system that is located at a known position. 
% For the human estimation we need to get from this wrenches but: 
% - multiplied by -1 (as the wrench applied on the human is exactly the 
%   opposite of the one excerted on the forceplate)
% - expressed in the frame of the human link in contact.
%
% This function computes the wrenches that each forceplate exerts on the link 
% in contact thank to the relative information between the human and the 
% forceplate, given by the setupUW.jpg file. 

%% Preliminary note: 
% The subject performs the task with the shoes on the plates. The ankle
% heigths to be considered are a sum of the real ankle height (coming from 
% subjectParamsFromData) + the fixed height of each shoe = 4.5 cm.
shoeHeight = 0.045; % in m

%% Change wrenches reference frames from Cortex to forceplates frames
% Transform forceplates wrenches from Cortex reference frame to the 
% forceplates ones.

% Extract wrenches and position from forceplate data 
fp1Wrench = forceplates.upsampled.FP1.wrenches';
fp2Wrench = forceplates.upsampled.FP2.wrenches';
pos = extractForceplatesPositionWrtCortex(filenameTrc);

gravityZero = iDynTree.Vector3();
gravityZero.zero();

% FP1 --> from cortex to FP1 frames
fp1PosWrtCortex = iDynTree.Position();
fp1PosWrtCortex.fromMatlab(pos.FP1');
fp1_R_cortex = iDynTree.Rotation();
fp1_R_cortex.fromMatlab ([ 1.0,  0.0,  0.0; ...
                           0.0, -1.0,  0.0; ...
                           0.0,  0.0, -1.0]);
fp1_T_cortex = iDynTree.Transform(fp1_R_cortex, fp1PosWrtCortex);
forceplates.upsampled.FP1.wrenchesInFp1frames = ...
            (fp1_T_cortex.asAdjointTransformWrench().toMatlab()*fp1Wrench);

% FP2 --> from cortex to FP2 frames
fp2PosWrtCortex = iDynTree.Position();
fp2PosWrtCortex.fromMatlab(pos.FP2');
fp2_R_cortex = iDynTree.Rotation();
fp2_R_cortex.fromMatlab ([ 1.0,  0.0,  0.0; ...
                           0.0, -1.0,  0.0; ...
                           0.0,  0.0, -1.0]);
fp2_T_cortex = iDynTree.Transform(fp2_R_cortex, fp2PosWrtCortex);
forceplates.upsampled.FP2.wrenchesInFp2frames = ...
            (fp2_T_cortex.asAdjointTransformWrench().toMatlab()*fp2Wrench);

%% Transform wrench from forceplates frames to human frames

% Transformation matrix T for forceplate 1 and 2 can be easily extracted 
% from the following sketches:
% - fixtureUW.pdf  --> for the position of the each foot wrt the related
%                      forceplate is located at the center of the rear
%                      sensor (assumption: on this point there is a
%                      reference frame oriented as in the foot).
%                      Do not forget to consider the height of the shoe!
% - rawSketch.jpg  --> for the rotation between each foot
%                      wrt the related forceplate

% FP1 --> from FP1 to human frames (related to leftFoot in UW setup)
leftSole_T_fp1Pos = iDynTree.Position();
leftSole_T_fp1Pos.fromMatlab([0.099; 0.063 ; 0]); % fixed, from fixtureUW.pdf
leftSole_R_fp1 = iDynTree.Rotation();
leftSole_R_fp1.fromMatlab ([ 0.0, -1.0,  0.0; ...
                            -1.0,  0.0,  0.0; ...
                             0.0,  0.0, -1.0]);
leftFoot_T_leftSolePos = iDynTree.Position();
leftFoot_T_leftSolePos.fromMatlab([0.0; 0.0; ...
             subjectParamsFromData.leftFootBoxOrigin(3) - shoeHeight])
leftFoot_T_fp1 = iDynTree.Transform(leftSole_R_fp1, ...
             leftFoot_T_leftSolePos + leftSole_T_fp1Pos);

% Transform the wrench in the appropriate frame and change the sign
forceplates.upsampled.FP1.humanLeftFootWrench = ...
    -1*(leftFoot_T_fp1.asAdjointTransformWrench().toMatlab()* ...
    forceplates.upsampled.FP1.wrenchesInFp1frames);

% FP2 --> from FP2 to human frames (related to rightFoot in UW setup)
rightSole_T_fp2Pos = iDynTree.Position();
rightSole_T_fp2Pos.fromMatlab([0.099; -0.064; 0]); % fixed, from fixtureUW.pdf
rightSole_R_fp2 = iDynTree.Rotation();
rightSole_R_fp2.fromMatlab ([ 0.0, -1.0,  0.0; ...
                             -1.0,  0.0,  0.0; ...
                              0.0,  0.0, -1.0]);
rightFoot_T_rightSolePos = iDynTree.Position();
rightFoot_T_rightSolePos.fromMatlab([0.0; 0.0; ...
             subjectParamsFromData.rightFootBoxOrigin(3) - shoeHeight])
rightFoot_T_fp2 = iDynTree.Transform(rightSole_R_fp2,...
             rightFoot_T_rightSolePos + rightSole_T_fp2Pos);

% calibrationForceplate1Pos = iDynTree.Position();
% calibrationForceplate1Pos.fromMatlab([0.0002; 0.0006; 0.0398]);

% rightSole_T_fpRot = iDynTree.Rotation();
% rightSole_T_fpRot.fromMatlab([ -1.0,  0.0,  0.0; ...
%                                 0.0,  1.0,  0.0; ...
%                                 0.0,  0.0, -1.0]);
% rigthSole_T_fpPos = iDynTree.Position();
% rigthSole_T_fpPos.fromMatlab([0.108; -0.107; 0]);

% rightFoot_T_rightSolePos = iDynTree.Position();
% rightFoot_T_rightSolePos.fromMatlab([0.0; 0.0; ...
%             -(subjectParamsFromData.rightFootBoxOrigin(3) + shoeHeight)] ); %to be verified!

% rightFoot_T_fp = iDynTree.Transform(rightSole_T_fpRot,...
%             rightFoot_T_rightSolePos + rigthSole_T_fpPos + calibrationForceplate1Pos);
%             
% % Extract wrenches from forceplate data 
% forceplate1Wrench(1:3,:) = forceplates.data.plateforms.plateform1.forces;
% forceplate1Wrench(4:6,:) = forceplates.data.plateforms.plateform1.moments;
% 
% %% Transform the wrench in the appropriate frame and change the sign 
% forceplates.processedData.humanRightFootWrench = -1*(rightFoot_T_fp.asAdjointTransformWrench().toMatlab()*forceplate1Wrench);
end
