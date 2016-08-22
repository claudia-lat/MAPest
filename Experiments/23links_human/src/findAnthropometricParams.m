function [subjectParams] = findAnthropometricParams(H, M)
%FINDANTHROPOMETRICPARAM finds the anthropometric parameters of the subject
% given his height H in [m] and mass M in [kg]. By exploiting the 
% anthropometric tables it is possible to estimate: 
%   - the geometric parameters of the solids (cylinders, boxes or spheres) 
%     representing each body segment;   
%   - mass of each body segment;
%   - momentum of inertia of each body segment.
%
% The geometric dimensions of each solid is defined as follow:
%   - box     :    alfa = width, beta = height, gamma = depth;
%   - cylinder:    radius and height (diameter case);
%   - sphere  :    radius (diameter in case)
%
% It returns also the homogeneous transformations between links as
% represented in the Xsens model. They will be useful for building the 
% OSIM model.

format longg %for decimal notation
%% FOOT (solid: box)
% Geometric dimensions
footAlfa      = 0.055 * H;
footBeta      = 0.058 * H;
footGamma     = 0.110 * H;
footHalfBeta  = 0.5 * footBeta;
footHalfGamma = 0.5 * footGamma;
anklePosX     = 0.25 * footGamma;
toePosX       = footHalfGamma + anklePosX;
% Mass and inertia
footMass      = 0.0130 *  M;
footIxx       = (footMass/12) * (footAlfa^2 + footBeta^2);
footIyy       = (footMass/12) * (footBeta^2 + footGamma^2);
footIzz       = (footMass/12) * (footGamma^2 + footAlfa^2);
%% TOE (solid: box)
% Geometric dimensions
toeAlfa      = 0.055 * H;
toeBeta      = 0.039 * H;
toeGamma     = 0.042 * H;
toeHalfGamma = 0.5 * toeGamma;
% Mass and inertia
toeMass      = 0.0015 * M;
toeIxx       = (toeMass/12) * (toeAlfa^2 + toeBeta^2);
toeIyy       = (toeMass/12) * (toeBeta^2 + toeGamma^2);
toeIzz       = (toeMass/12) * (toeGamma^2 + toeAlfa^2);
%% LOWER LEG (solid: cylinder)
% Geometric dimensions
lowerLegHeight     = 0.246 * H;
lowerLegHalfHeight = 0.5 * lowerLegHeight;
lowerLegRadius     = 0.25 * footGamma; %arbitrarily defined as 1/4 of the foot depth
lowerLegDiameter   = 0.5 * footGamma;
% Mass and inertia
lowerLegMass       = 0.0465 * M;
lowerLegIxx        = (lowerLegMass/12) * (3 * lowerLegRadius^2 + lowerLegHeight^2);
lowerLegIyy        = (lowerLegMass/12) * (3 * lowerLegRadius^2 + lowerLegHeight^2);
lowerLegIzz        = (lowerLegMass/2) * (lowerLegRadius^2);
%% TORSO (solid: box)
% Geometric dimensions
torsoAlfa     = 0.5 * (0.191 * H+0.259 * H); %arbitrarily defined as the mean of hip width and shoulder width
torsoBeta     = 0.288 * H;
torsoGamma    = 0.75 * footGamma; %arbitrarily defined as 3/4 of the foot depth
torsoHalfBeta = 0.5 * torsoBeta;
% Mass and inertia
torsoMass     = 0.497 * M;
torsoIxx      = (torsoMass/12) * (torsoAlfa^2 + torsoBeta^2);
torsoIyy      = (torsoMass/12) * (torsoBeta^2 + torsoGamma^2);
torsoIzz      = (torsoMass/12) * (torsoGamma^2 + torsoAlfa^2);
%% UPPER LEG (solid: cylinder)
% Geometric dimensions
upperLegHeight     = 0.245 * H;
upperLegHalfHeight = 0.5 * upperLegHeight;
upperLegDiameter   = 0.5 * (2 * lowerLegRadius+0.5 * torsoAlfa); %arbitrarily defined as the mean of the width of the lower leg and the half width of the torso
upperLegRadius     = 0.5 * upperLegDiameter;
HipPosY            = 0.5 * (torsoAlfa-2 * upperLegRadius); % distance between right hip and the mid point of the torso along y
HipDoublePosY = 2 * HipPosY;
% Mass and inertia
upperLegMass       = 0.100 * M;
upperLegIxx        = (upperLegMass/12) * (3 * upperLegRadius^2 + upperLegHeight^2);
upperLegIyy        = (upperLegMass/12) * (3 * upperLegRadius^2 + upperLegHeight^2);
upperLegIzz        = (upperLegMass/2) * (upperLegRadius^2);
%% PELVIS (solid: box)
% Geometric dimensions
pelvisAlfa     = 0.191 * H; 
pelvisBeta     = 0.1887 * torsoBeta;
pelvisGamma    = footGamma; %arbitrarily defined as the foot depth
pelvisHalfBeta = 0.5 * pelvisBeta;
% Mass and inertia
pelvisMass     = 0.1750 * torsoMass;
pelvisIxx      = (pelvisMass/12) * (pelvisAlfa^2 + pelvisBeta^2);
pelvisIyy      = (pelvisMass/12) * (pelvisBeta^2 + pelvisGamma^2);
pelvisIzz      = (pelvisMass/12) * (pelvisGamma^2 + pelvisAlfa^2);
%% L5 (solid: cylinder)
% Geometric dimensions
L5DiameterX  = torsoGamma; 
L5RadiusX    = L5DiameterX/2;
L5DiameterY  = 0.174 * H; 
L5RadiusY    = L5DiameterY/2;
L5Height     = 0.2004 * torsoBeta;
L5HalfHeight = 0.5 * L5Height;
% Mass and inertia
L5Mass       = 0.0998 * torsoMass;
L5Ixx        = (L5Mass/12) * (3 * L5RadiusY^2 + L5Height^2);
L5Iyy        = (L5Mass/12) * (3 * L5RadiusX^2 + L5Height^2);
L5Izz        = (L5Mass/2) * (L5RadiusX * L5RadiusY);
%% L3 (solid: cylinder)
% Geometric dimensions
L3DiameterX  = torsoGamma; 
L3RadiusX    = L3DiameterX/2;
L3DiameterY  = 0.174 * H; 
L3RadiusY    = L3DiameterY/2;
L3Height     = 0.1810 * torsoBeta;
L3HalfHeight = 0.5 * L3Height;
% Mass and inertia
L3Mass       = 0.0901 * torsoMass;
L3Ixx        = (L3Mass/12) * (3 * L3RadiusY^2 + L3Height^2);
L3Iyy        = (L3Mass/12) * (3 * L3RadiusX^2 + L3Height^2);
L3Izz        = (L3Mass/2) * (L3RadiusX * L3RadiusY);
%% T12 (solid: cylinder)
% Geometric dimensions
T12DiameterX  = torsoGamma; 
T12RadiusX    = T12DiameterX/2;
T12DiameterY  = 0.174 * H; 
T12RadiusY    = T12DiameterY/2;
T12Height     = 0.1810 * torsoBeta;
T12HalfHeight = 0.5 * T12Height;
% Mass and inertia
T12Mass       = 0.0901 * torsoMass;
T12Ixx        = (T12Mass/12) * (3 * T12RadiusY^2 + T12Height^2);
T12Iyy        = (T12Mass/12) * (3 * T12RadiusX^2 + T12Height^2);
T12Izz        = (T12Mass/2) * (T12RadiusX * T12RadiusY);
%% T8 (solid: box)
% Geometric dimensions
T8Alfa     = 0.0553 * H; 
T8Beta     = 0.2489 * torsoBeta;
T8Gamma    = torsoGamma; 
T8HalfBeta = 0.5 * T8Beta;
T8HalfAlfa = 0.5 * T8Alfa;
% Mass and inertia
T8Mass     = 0.0501 * torsoMass;
T8Ixx      = (T8Mass/12) * (T8Alfa^2 + T8Beta^2);
T8Iyy      = (T8Mass/12) * (T8Beta^2 + T8Gamma^2);
T8Izz      = (T8Mass/12) * (T8Gamma^2 + T8Alfa^2);
%% SHOULDER (solid: cylinder)
% Geometric dimensions
shoulderDiameter   = 0.1019 * torsoBeta; 
shoulderRadius     = 0.5 * shoulderDiameter;
shoulderHeight     = 0.129 * H;
shoulderHalfHeight = 0.5 * shoulderHeight;
shoulderPosZ       = T8HalfBeta - shoulderRadius; %distance between the centre of t8 and the joint C7shoulder, along z
shoulderUrdfPosZ   = T8Beta - shoulderRadius; %distance between the joint jT9T8 and the joint C7shoulder, along z
% Mass and inertia
shoulderMass       = 0.2474 * torsoMass;
shoulderIxx        = (shoulderMass/12) * (3 * shoulderRadius^2 + shoulderHeight^2);
shoulderIyy        = (shoulderMass/2) * (shoulderRadius^2);
shoulderIzz        = (shoulderMass/12) * (3 * shoulderRadius^2 + shoulderHeight^2);
%% UPPER ARM (solid: cylinder)
% Geometric dimensions
upperArmRadius     = lowerLegRadius; %radius of tha arm define equal to the radius of the lower leg
upperArmDiameter   = upperArmRadius * 2;
upperArmHeight     = 0.186 * H;
upperArmHalfHeight = 0.5 * upperArmHeight;
armPosZ            = upperArmHalfHeight - shoulderRadius; %distance between the centre of the upper arm and the joint shoulder, along z
armUrdfPosZ        = upperArmHalfHeight + armPosZ; %distance between the elbow and the joint shoulder, along z
% Mass and inertia
upperArmMass       = 0.028 * M;
upperArmIxx        = (upperArmMass/12) * (3 * upperArmRadius^2 + upperArmHeight^2);
upperArmIyy        = (upperArmMass/12) * (3 * upperArmRadius^2 + upperArmHeight^2);
upperArmIzz        = (upperArmMass/2) * (upperArmRadius^2);
%% FOREARM (solid: cylinder)
% Geometric dimensions
foreArmRadius     = 0.75 * upperArmRadius; %arbitrarily defined as the 3/4 of the upper arm radius
foreArmDiameter   = foreArmRadius * 2;
foreArmHeight     = 0.146 * H;
foreArmHalfHeight = 0.5 * foreArmHeight;
% Mass and inertia
foreArmMass       = 0.016 * M;
foreArmIxx        = (foreArmMass/12) * (3 * foreArmRadius^2 + foreArmHeight^2);
foreArmIyy        = (foreArmMass/12) * (3 * foreArmRadius^2 + foreArmHeight^2);
foreArmIzz        = (foreArmMass/2) * (foreArmRadius^2);
%% HAND (solid: box)
% Geometric dimensions
handGamma    = foreArmDiameter; %arbitrarily defined as the width of the forearm
handBeta     = 0.108 * H;
handAlfa     = foreArmRadius; %arbitrarily defined as the half depth of the forearm
handHalfBeta = 0.5 * handBeta;
% Mass and inertia
handMass     = 0.006 * M;
handIxx      = (handMass/12) * (handGamma^2 + handBeta^2);
handIyy      = (handMass/12) * (handBeta^2 + handAlfa^2);
handIzz      = (handMass/12) * (handAlfa^2 + handGamma^2);
%% HEAD (solid: sphere)
% Geometric dimensions
headDiameter = 0.13 * H; %Height of the head
headRadius   = 0.5 * headDiameter;
% Mass and inertia
headMass     = 0.006 * M;
headIxx      = (2 * headMass/5) * (headRadius^2);
headIyy      = (2 * headMass/5) * (headRadius^2);
headIzz      = (2 * headMass/5) * (headRadius^2);
%% NECK (solid: cylinder)
% Geometric dimensions
neckRadius     = 0.5 * headRadius; %arbitrarily defined as the half of the head radius
neckDiameter   = neckRadius * 2;
neckHeight     = 0.066 * H;
neckHalfHeight = 0.5 * neckHeight;
% Mass and inertia
neckMass       = 0.016 * M;
neckIxx        = (neckMass/12) * (3 * neckRadius^2 + neckHeight^2);
neckIyy        = (neckMass/12) * (3 * neckRadius^2 + neckHeight^2);
neckIzz        = (neckMass/2) * (neckRadius^2);
%% Homogeneous transformations
pos = [-toeHalfGamma; 0; 0];
ballFoot_H_toe = [  eye(3)    pos
                  zeros(1,3)   1 ];
pos = [-anklePosX;0;footHalfBeta];              
ankle_H_foot= [ eye(3)    pos
               zeros(1,3)  1 ]; 
pos = [0;0;lowerLegHalfHeight];               
knee_H_lowerLeg = [ eye(3)    pos
                   zeros(1,3)  1 ];
pos = [0;0;upperLegHalfHeight];               
hip_H_upperLeg = [ eye(3)    pos
                  zeros(1,3)  1 ];
pos = [0;0;-0.5*pelvisHalfBeta];                
hipOrigin_H_pelvis = [ eye(3)    pos
                      zeros(1,3)  1 ];
pos = [0; 0; -L5HalfHeight];                  
jL5S1_H_l5 = [ eye(3)    pos
              zeros(1,3)  1 ]; 
pos = [0; 0; -L3HalfHeight]; 
jL4L3_H_l3 = [ eye(3)    pos
              zeros(1,3)  1 ]; 
pos = [0; 0; -T12HalfHeight];
jL1T12_H_T12 = [ eye(3)    pos
                zeros(1,3)  1 ]; 
pos = [0; 0; -T8HalfBeta]; 
jT9T8_H_T8 = [ eye(3)    pos
              zeros(1,3)  1 ];
pos = [0; 0; -neckHalfHeight];         
jT1C7_H_neck = [ eye(3)    pos
                zeros(1,3)  1 ];
pos = [0; 0; -headRadius];  
jC1Head_H_head = [ eye(3)    pos
                  zeros(1,3)  1 ];
pos = [0; shoulderHalfHeight; 0];
jRightC7Shoulder_H_rightShoulder = [ eye(3)    pos
                                    zeros(1,3)  1 ];
pos = [0; -shoulderHalfHeight; 0];
jLeftC7Shoulder_H_leftShoulder = [ eye(3)    pos
                                  zeros(1,3)  1 ];                             
pos = [0; 0; armPosZ];
rotX = iDynTree.Rotation.RotX(pi/2).toMatlab;
jRightShoulder_H_rightUpperArm = [ rotX      pos
                                  zeros(1,3)  1 ];    
rotX = iDynTree.Rotation.RotX(-pi/2).toMatlab;
jLeftShoulder_H_leftUpperArm = [ rotX      pos
                                zeros(1,3)  1 ]; 
rotX_c  = iDynTree.Rotation.RotX(-pi/2).toMatlab;
rotX_cc = iDynTree.Rotation.RotX(pi/2).toMatlab;                     
pos = [0; 0; foreArmHalfHeight];
jRightElbow_H_rightForeArm = [ rotX_cc   pos
                              zeros(1,3)  1 ];    
jLeftElbow_H_leftForeArm = [ rotX_c    pos
                            zeros(1,3)  1 ];                               
pos = [0; 0; foreArmHalfHeight];
jRightElbow_H_rightForeArm = [ rotX_cc   pos
                              zeros(1,3)  1 ];    
jLeftElbow_H_leftForeArm = [ rotX_c    pos
                            zeros(1,3)  1 ];
pos = [0; 0; handHalfBeta];
jRightWrist_H_rightHand = [ rotX_cc   pos
                           zeros(1,3)  1 ];    
jLeftWrist_H_leftHand = [ rotX_c    pos
                         zeros(1,3)  1 ];
%% Save parameters
% FOOT
subjectParams.footAlfa      = footAlfa;
subjectParams.footBeta      = footBeta;
subjectParams.footGamma     = footGamma;
subjectParams.footHalfBeta  = footHalfBeta;
subjectParams.footHalfGamma = footHalfGamma;
subjectParams.anklePosX     = anklePosX;
subjectParams.toePosX       = toePosX;
subjectParams.footMass      = footMass;
subjectParams.footIxx       = footIxx;
subjectParams.footIyy       = footIyy; 
subjectParams.footIzz       = footIzz;
% TOE
subjectParams.toeAlfa      = toeAlfa;
subjectParams.toeBeta      = toeBeta;
subjectParams.toeGamma     = toeGamma;
subjectParams.toeHalfGamma = toeHalfGamma;
subjectParams.toeMass      = toeMass;
subjectParams.toeIxx       = toeIxx;
subjectParams.toeIyy       = toeIyy; 
subjectParams.toeIzz       = toeIzz;
% LOWER LEG
subjectParams.lowerLegHeight     = lowerLegHeight;
subjectParams.lowerLegRadius     = lowerLegRadius;
subjectParams.lowerLegDiameter   = lowerLegDiameter;
subjectParams.lowerLegHalfHeight = lowerLegHalfHeight;
subjectParams.lowerLegMass       = lowerLegMass;
subjectParams.lowerLegIxx        = lowerLegIxx;
subjectParams.lowerLegIyy        = lowerLegIyy;
subjectParams.lowerLegIzz        = lowerLegIzz;
% UPPER LEG
subjectParams.upperLegHeight     = upperLegHeight;
subjectParams.upperLegRadius     = upperLegRadius;
subjectParams.upperLegDiameter   = upperLegDiameter;
subjectParams.upperLegHalfHeight = upperLegHalfHeight;
subjectParams.HipPosY            = HipPosY;
subjectParams.HipDoublePosY      = HipDoublePosY;
subjectParams.upperLegMass       = upperLegMass;
subjectParams.upperLegIxx        = upperLegIxx;
subjectParams.upperLegIyy        = upperLegIyy;
subjectParams.upperLegIzz        = upperLegIzz;
% TORSO
subjectParams.torsoAlfa     = torsoAlfa;
subjectParams.torsoBeta     = torsoBeta;
subjectParams.torsoGamma    = torsoGamma;
subjectParams.torsoHalfBeta = torsoHalfBeta;
subjectParams.torsoMass     = torsoMass;
subjectParams.torsoIxx      = torsoIxx;
subjectParams.torsoIyy      = torsoIyy;
subjectParams.torsoIzz      = torsoIzz;
% PELVIS
subjectParams.pelvisAlfa     = pelvisAlfa;
subjectParams.pelvisBeta     = pelvisBeta;
subjectParams.pelvisGamma    = pelvisGamma;
subjectParams.pelvisHalfBeta = pelvisHalfBeta;
subjectParams.pelvisMass     = pelvisMass;
subjectParams.pelvisIxx      = pelvisIxx;
subjectParams.pelvisIyy      = pelvisIyy;
subjectParams.pelvisIzz      = pelvisIzz;
% L5
subjectParams.L5Height     = L5Height;
subjectParams.L5RadiusX    = L5RadiusX;
subjectParams.L5RadiusY    = L5RadiusY;
subjectParams.L5DiameterX  = L5DiameterX;
subjectParams.L5DiameterY  = L5DiameterY;
subjectParams.L5HalfHeight = L5HalfHeight;
subjectParams.L5Mass       = L5Mass;
subjectParams.L5Ixx        = L5Ixx;
subjectParams.L5Iyy        = L5Iyy;
subjectParams.L5Izz        = L5Izz;
% L3
subjectParams.L3Height     = L3Height;
subjectParams.L3RadiusX    = L3RadiusX;
subjectParams.L3RadiusY    = L3RadiusY;
subjectParams.L3DiameterX  = L3DiameterX;
subjectParams.L3DiameterY  = L3DiameterY;
subjectParams.L3HalfHeight = L3HalfHeight;
subjectParams.L3Mass       = L3Mass;
subjectParams.L3Ixx        = L3Ixx;
subjectParams.L3Iyy        = L3Iyy;
subjectParams.L3Izz        = L3Izz;
% T12
subjectParams.T12Height     = T12Height;
subjectParams.T12RadiusX    = T12RadiusX;
subjectParams.T12RadiusY    = T12RadiusY;
subjectParams.T12DiameterX  = T12DiameterX;
subjectParams.T12DiameterY  = T12DiameterY;
subjectParams.T12HalfHeight = T12HalfHeight;
subjectParams.T12Mass       = T12Mass;
subjectParams.T12Ixx        = T12Ixx;
subjectParams.T12Iyy        = T12Iyy;
subjectParams.T12Izz        = T12Izz;
% T8
subjectParams.T8Alfa     = T8Alfa;
subjectParams.T8Beta     = T8Beta;
subjectParams.T8Gamma    = T8Gamma;
subjectParams.T8HalfBeta = T8HalfBeta;
subjectParams.T8HalfAlfa = T8HalfAlfa;
subjectParams.T8Mass     = T8Mass;
subjectParams.T8Ixx      = T8Ixx;
subjectParams.T8Iyy      = T8Iyy;
subjectParams.T8Izz      = T8Izz;
% SHOULDER
subjectParams.shoulderRadius     = shoulderRadius;
subjectParams.shoulderHeight     = shoulderHeight;
subjectParams.shoulderHalfHeight = shoulderHalfHeight;
subjectParams.shoulderDiameter   = shoulderDiameter;
subjectParams.shoulderPosZ       = shoulderPosZ;
subjectParams.shoulderUrdfPosZ   = shoulderUrdfPosZ;
subjectParams.shoulderMass       = shoulderMass;
subjectParams.shoulderIxx        = shoulderIxx;
subjectParams.shoulderIyy        = shoulderIyy;
subjectParams.shoulderIzz        = shoulderIzz;
% UPPER ARM
subjectParams.upperArmRadius     = upperArmRadius;
subjectParams.upperArmHeight     = upperArmHeight;
subjectParams.upperArmHalfHeight = upperArmHalfHeight;
subjectParams.upperArmDiameter   = upperArmDiameter;
subjectParams.armPosZ            = armPosZ;
subjectParams.armUrdfPosZ        = armUrdfPosZ;
subjectParams.upperArmMass       = upperArmMass;
subjectParams.upperArmIxx        = upperArmIxx;
subjectParams.upperArmIyy        = upperArmIyy;
subjectParams.upperArmIzz        = upperArmIzz;
% FOREARM
subjectParams.foreArmRadius     = foreArmRadius;
subjectParams.foreArmDiameter   = foreArmDiameter;
subjectParams.foreArmHeight     = foreArmHeight;
subjectParams.foreArmHalfHeight = foreArmHalfHeight;
subjectParams.foreArmMass       = foreArmMass;
subjectParams.foreArmIxx        = foreArmIxx;
subjectParams.foreArmIyy        = foreArmIyy;
subjectParams.foreArmIzz        = foreArmIzz;
% HAND
subjectParams.handAlfa     = handGamma;
subjectParams.handBeta     = handBeta;
subjectParams.handGamma    = handGamma;
subjectParams.handHalfBeta = handHalfBeta;
subjectParams.handMass     = handMass;
subjectParams.handIxx      = handIxx;
subjectParams.handIyy      = handIyy;
subjectParams.handIzz      = handIzz;
% NECK
subjectParams.neckRadius     = neckRadius;
subjectParams.neckDiameter   = neckDiameter;
subjectParams.neckHeight     = neckHeight;
subjectParams.neckHalfHeight = neckHalfHeight;
subjectParams.neckMass       = neckMass;
subjectParams.neckIxx        = neckIxx;
subjectParams.neckIyy        = neckIyy;
subjectParams.neckIzz        = neckIzz;
% HEAD
subjectParams.headDiameter = headDiameter;
subjectParams.headRadius   = headRadius;
subjectParams.headMass     = headMass;
subjectParams.headIxx      = headIxx;
subjectParams.headIyy      = headIyy; 
subjectParams.headIzz      = headIzz;
% HOMOGENEOUS TRANSFORMATIONS
subjectParams.ballFoot_H_toe                   = ballFoot_H_toe;
subjectParams.ankle_H_foot                     = ankle_H_foot;
subjectParams.knee_H_lowerLeg                  = knee_H_lowerLeg;
subjectParams.hip_H_upperLeg                   = hip_H_upperLeg;
subjectParams.hipOrigin_H_pelvis               = hipOrigin_H_pelvis;
subjectParams.jL5S1_H_l5                       = jL5S1_H_l5;
subjectParams.jL4L3_H_l3                       = jL4L3_H_l3;
subjectParams.jL1T12_H_T12                     = jL1T12_H_T12;
subjectParams.jT9T8_H_T8                       = jT9T8_H_T8;
subjectParams.jT1C7_H_neck                     = jT1C7_H_neck;
subjectParams.jC1Head_H_head                   = jC1Head_H_head;
subjectParams.jRightC7Shoulder_H_rightShoulder = jRightC7Shoulder_H_rightShoulder;
subjectParams.jLeftC7Shoulder_H_leftShoulder   = jLeftC7Shoulder_H_leftShoulder;
subjectParams.jRightShoulder_H_rightUpperArm   = jRightShoulder_H_rightUpperArm;
subjectParams.jLeftShoulder_H_leftUpperArm     = jLeftShoulder_H_leftUpperArm;
subjectParams.jRightElbow_H_rightForeArm       = jRightElbow_H_rightForeArm;
subjectParams.jLeftElbow_H_leftForeArm         = jLeftElbow_H_leftForeArm;
subjectParams.jRightWrist_H_rightHand          = jRightWrist_H_rightHand;
subjectParams.jLeftWrist_H_leftHand            = jLeftWrist_H_leftHand;
end
