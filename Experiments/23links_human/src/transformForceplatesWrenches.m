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
% forceplate, given by the rawSketch.jpg file. 

%% Preliminary note: 
% The subject performs the task with the shoes on the plates.
% Fixed height of each shoe = 0.060 m

gravityZero = iDynTree.Vector3();
gravityZero.zero();

%% Extract wrenches and position from forceplate data 
fp1Wrench = forceplates.upsampled.FP1.wrenches';
fp2Wrench = forceplates.upsampled.FP2.wrenches';

%% Transform wrenches from forceplates frames to human frames
% Useful information for these transformation are in:
% - rawSketch.jpg  --> for the rotation between each foot wrt the related 
%                      forceplate;
% - fixtureUW.pdf  --> for the position of the each foot wrt the related
%                      forceplate taht is located at the center of the rear
%                      sensor (assumption: on this point there is a
%                      reference frame oriented as in the foot).

% ---- FP1 --> transform FP1 data from FP1 frame to leftFoot frame
% We want: leftFoot_f_fp1 starting from fp1_f_fp1
leftSole_R_fp1 = iDynTree.Rotation(); % ==leftFoot_R_fp1 ==leftHeel_R_fp1
leftSole_R_fp1.fromMatlab ([ 0.0, -1.0,  0.0; ...
                            -1.0,  0.0,  0.0; ...
                             0.0,  0.0, -1.0]);
leftSole_T_fp1Pos = iDynTree.Position();
fp1SeenFromLeftSole = [0.099; 0.063 ; 0];
leftSole_T_fp1Pos.fromMatlab(fp1SeenFromLeftSole);

leftHeel_T_leftSolePos = iDynTree.Position();
leftSoleSeenFromLeftHeel = [0.037; 0 ; -0.060];
leftHeel_T_leftSolePos.fromMatlab(leftSoleSeenFromLeftHeel);

leftFoot_T_leftHeelPos = iDynTree.Position();
leftHeelSeenFromLeftFoot = subjectParamsFromData.pLeftHeelFoot;
leftFoot_T_leftHeelPos.fromMatlab(leftHeelSeenFromLeftFoot);

leftFoot_T_fp1 = iDynTree.Transform(leftSole_R_fp1, ...
             leftFoot_T_leftHeelPos + leftHeel_T_leftSolePos + leftSole_T_fp1Pos);

% Transform the wrench in the proper frame and change the sign
forceplates.upsampled.FP1.humanLeftFootWrench = ...
              -1*(leftFoot_T_fp1.asAdjointTransformWrench().toMatlab()* ...
              fp1Wrench);

% ------WRONG CODE
% leftSole_T_fp1Pos = iDynTree.Position();
% fp1SeenFromLeftSole = [0.099; 0.063 ; 0];
% leftSole_T_fp1Pos.fromMatlab(fp1SeenFromLeftSole);
% leftFoot_T_leftSolePos = iDynTree.Position();
% leftSoleSeenFromLeftFoot = [0.0; 0.0; ...
%                     subjectParamsFromData.leftFootBoxOrigin(3) - shoeHeight];
% leftFoot_T_leftSolePos.fromMatlab(leftSoleSeenFromLeftFoot);
% leftSole_R_fp1 = iDynTree.Rotation(); % ==leftFoot_R_fp1
% leftSole_R_fp1.fromMatlab ([ 0.0, -1.0,  0.0; ...
%                             -1.0,  0.0,  0.0; ...
%                              0.0,  0.0, -1.0]);
% leftFoot_T_fp1 = iDynTree.Transform(leftSole_R_fp1, ...
%              leftFoot_T_leftSolePos + leftSole_T_fp1Pos);
% ------WRONG CODE


% ---- FP2 --> transform FP2 data from FP2 frame to rightFoot frame
% We want: rightFoot_f_fp2 starting from fp2_f_fp2
rightSole_R_fp2 = iDynTree.Rotation(); % ==rightFoot_R_fp2 ==rightHeel_R_fp2
rightSole_R_fp2.fromMatlab ([ 0.0, -1.0,  0.0; ...
                             -1.0,  0.0,  0.0; ...
                              0.0,  0.0, -1.0]);
rightSole_T_fp2Pos = iDynTree.Position();
fp2SeenFromRightSole = [0.099; -0.064 ; 0];
rightSole_T_fp2Pos.fromMatlab(fp2SeenFromRightSole);

rightHeel_T_rightSolePos = iDynTree.Position();
rigthSoleSeenFromRightHeel = [0.037; 0 ; -0.060];
rightHeel_T_rightSolePos.fromMatlab(rigthSoleSeenFromRightHeel);

rightFoot_T_rightHeelPos = iDynTree.Position();
rightHeelSeenFromRightFoot = subjectParamsFromData.pRightHeelFoot;
rightFoot_T_rightHeelPos.fromMatlab(rightHeelSeenFromRightFoot);

rightFoot_T_fp2 = iDynTree.Transform(rightSole_R_fp2, ...
             rightFoot_T_rightHeelPos + rightHeel_T_rightSolePos + rightSole_T_fp2Pos);

% transform the wrench in the proper frame and change the sign
forceplates.upsampled.FP2.humanRightFootWrench = ...
              -1*(rightFoot_T_fp2.asAdjointTransformWrench().toMatlab()* ...
              fp2Wrench);

end
