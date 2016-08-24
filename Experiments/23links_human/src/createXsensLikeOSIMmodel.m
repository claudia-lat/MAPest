function [osimModelTemplate] = createXsensLikeOSIMmodel(subjectParams, suit, filename)
%CREATEXSENSLIKEOSIMMODEL generates an OSIM model of the subject. 
%
% Inputs : 
% -  subjectParams  : anthropometric parameters;
% -  suit           : suit data stuct. Point in suit They represents 
%                     location of fake markerss in global suit
%                     reference frame;
% -  filename       : (optional) allows to save the file.osim in a folder 
%                      called 'Models'.  

    
osimModelTemplate = fileread('XSensModelStyle_OSIMtemplate.osim');
% FOOT
osimModelTemplate = strrep(osimModelTemplate,'FOOTALFA',sprintf('%.10f',subjectParams.footAlfa));
osimModelTemplate = strrep(osimModelTemplate,'FOOTBETA',sprintf('%.10f',subjectParams.footBeta));
osimModelTemplate = strrep(osimModelTemplate,'FOOTGAMMA',sprintf('%.10f',subjectParams.footGamma));
osimModelTemplate = strrep(osimModelTemplate,'FOOTHALFBETA',sprintf('%.10f',subjectParams.footHalfBeta));
osimModelTemplate = strrep(osimModelTemplate,'FOOTHALFGAMMA',sprintf('%.10f',subjectParams.footHalfGamma));
osimModelTemplate = strrep(osimModelTemplate,'ANKLEPOSX',sprintf('%.10f',subjectParams.anklePosX));
osimModelTemplate = strrep(osimModelTemplate,'TOEPOSX',sprintf('%.10f',subjectParams.toePosX));
osimModelTemplate = strrep(osimModelTemplate,'FOOTMASS',sprintf('%.10f',subjectParams.footMass));
osimModelTemplate = strrep(osimModelTemplate,'FOOTINERTIAIXX',sprintf('%.10f',subjectParams.footIxx));
osimModelTemplate = strrep(osimModelTemplate,'FOOTINERTIAIYY',sprintf('%.10f',subjectParams.footIyy));
osimModelTemplate = strrep(osimModelTemplate,'FOOTINERTIAIZZ',sprintf('%.10f',subjectParams.footIzz));
% TOE
osimModelTemplate = strrep(osimModelTemplate,'TOEALFA',sprintf('%.10f',subjectParams.toeAlfa));
osimModelTemplate = strrep(osimModelTemplate,'TOEBETA',sprintf('%.10f',subjectParams.toeBeta));
osimModelTemplate = strrep(osimModelTemplate,'TOEGAMMA',sprintf('%.10f',subjectParams.toeGamma));
osimModelTemplate = strrep(osimModelTemplate,'TOEHALFGAMMA',sprintf('%.10f',subjectParams.toeHalfGamma));
osimModelTemplate = strrep(osimModelTemplate,'TOEMASS',sprintf('%.10f',subjectParams.toeMass));
osimModelTemplate = strrep(osimModelTemplate,'TOEINERTIAIXX',sprintf('%.10f',subjectParams.toeIxx));
osimModelTemplate = strrep(osimModelTemplate,'TOEINERTIAIYY',sprintf('%.10f',subjectParams.toeIyy));
osimModelTemplate = strrep(osimModelTemplate,'TOEINERTIAIZZ',sprintf('%.10f',subjectParams.toeIzz));
% LOWER LEG
osimModelTemplate = strrep(osimModelTemplate,'LOWERLEGHEIGHT',sprintf('%.10f',subjectParams.lowerLegHeight));
osimModelTemplate = strrep(osimModelTemplate,'LOWERLEGRADIUS',sprintf('%.10f',subjectParams.lowerLegRadius));
osimModelTemplate = strrep(osimModelTemplate,'LOWERLEGHALFHEIGHT',sprintf('%.10f',subjectParams.lowerLegHalfHeight));
osimModelTemplate = strrep(osimModelTemplate,'LOWERLEGDIAMETER',sprintf('%.10f',subjectParams.lowerLegDiameter));
osimModelTemplate = strrep(osimModelTemplate,'LOWERLEGMASS',sprintf('%.10f',subjectParams.lowerLegMass));
osimModelTemplate = strrep(osimModelTemplate,'LOWERLEGINERTIAIXX',sprintf('%.10f',subjectParams.lowerLegIxx));
osimModelTemplate = strrep(osimModelTemplate,'LOWERLEGINERTIAIYY',sprintf('%.10f',subjectParams.lowerLegIyy));
osimModelTemplate = strrep(osimModelTemplate,'LOWERLEGINERTIAIZZ',sprintf('%.10f',subjectParams.lowerLegIzz));
% UPPER LEG
osimModelTemplate = strrep(osimModelTemplate,'UPPERLEGHEIGHT',sprintf('%.10f',subjectParams.upperLegHeight));
osimModelTemplate = strrep(osimModelTemplate,'UPPERLEGRADIUS',sprintf('%.10f',subjectParams.upperLegRadius));
osimModelTemplate = strrep(osimModelTemplate,'UPPERLEGDIAMETER',sprintf('%.10f',subjectParams.upperLegDiameter));
osimModelTemplate = strrep(osimModelTemplate,'UPPERLEGHALFHEIGHT',sprintf('%.10f',subjectParams.upperLegHalfHeight));
osimModelTemplate = strrep(osimModelTemplate,'HIPPOSY',sprintf('%.10f',subjectParams.HipPosY));
osimModelTemplate = strrep(osimModelTemplate,'HIPDOUBLEPOSY',sprintf('%.10f',subjectParams.HipDoublePosY));
osimModelTemplate = strrep(osimModelTemplate,'UPPERLEGMASS',sprintf('%.10f',subjectParams.upperLegMass));
osimModelTemplate = strrep(osimModelTemplate,'UPPERLEGINERTIAIXX',sprintf('%.10f',subjectParams.upperLegIxx));
osimModelTemplate = strrep(osimModelTemplate,'UPPERLEGINERTIAIYY',sprintf('%.10f',subjectParams.upperLegIyy));
osimModelTemplate = strrep(osimModelTemplate,'UPPERLEGINERTIAIZZ',sprintf('%.10f',subjectParams.upperLegIzz));
% TORSO
osimModelTemplate = strrep(osimModelTemplate,'TORSOALFA',sprintf('%.10f',subjectParams.torsoAlfa));
osimModelTemplate = strrep(osimModelTemplate,'TORSOBETA',sprintf('%.10f',subjectParams.torsoBeta));
osimModelTemplate = strrep(osimModelTemplate,'TORSOGAMMA',sprintf('%.10f',subjectParams.torsoGamma));
osimModelTemplate = strrep(osimModelTemplate,'TORSOHALFBETA',sprintf('%.10f',subjectParams.torsoHalfBeta));
osimModelTemplate = strrep(osimModelTemplate,'TORSOMASS',sprintf('%.10f',subjectParams.torsoMass));
osimModelTemplate = strrep(osimModelTemplate,'TORSOINERTIAIXX',sprintf('%.10f',subjectParams.torsoIxx));
osimModelTemplate = strrep(osimModelTemplate,'TORSOINERTIAIYY',sprintf('%.10f',subjectParams.torsoIyy));
osimModelTemplate = strrep(osimModelTemplate,'TORSOINERTIAIZZ',sprintf('%.10f',subjectParams.torsoIzz));
% PELVIS
osimModelTemplate = strrep(osimModelTemplate,'PELVISALFA',sprintf('%.10f',subjectParams.pelvisAlfa));
osimModelTemplate = strrep(osimModelTemplate,'PELVISBETA',sprintf('%.10f',subjectParams.pelvisBeta));
osimModelTemplate = strrep(osimModelTemplate,'PELVISGAMMA',sprintf('%.10f',subjectParams.pelvisGamma));
osimModelTemplate = strrep(osimModelTemplate,'PELVISHALFBETA',sprintf('%.10f',subjectParams.pelvisHalfBeta));
osimModelTemplate = strrep(osimModelTemplate,'PELVISMASS',sprintf('%.10f',subjectParams.pelvisMass));
osimModelTemplate = strrep(osimModelTemplate,'PELVISINERTIAIXX',sprintf('%.10f',subjectParams.pelvisIxx));
osimModelTemplate = strrep(osimModelTemplate,'PELVISINERTIAIYY',sprintf('%.10f',subjectParams.pelvisIyy));
osimModelTemplate = strrep(osimModelTemplate,'PELVISINERTIAIZZ',sprintf('%.10f',subjectParams.pelvisIzz));
% L5
osimModelTemplate = strrep(osimModelTemplate,'L5HEIGHT',sprintf('%.10f',subjectParams.L5Height));
osimModelTemplate = strrep(osimModelTemplate,'L5RADIUSX',sprintf('%.10f',subjectParams.L5RadiusX));
osimModelTemplate = strrep(osimModelTemplate,'L5DIAMETERX',sprintf('%.10f',subjectParams.L5DiameterX));
osimModelTemplate = strrep(osimModelTemplate,'L5RADIUSY',sprintf('%.10f',subjectParams.L5RadiusY));
osimModelTemplate = strrep(osimModelTemplate,'L5DIAMETERY',sprintf('%.10f',subjectParams.L5DiameterY));
osimModelTemplate = strrep(osimModelTemplate,'L5HALFHEIGHT',sprintf('%.10f',subjectParams.L5HalfHeight));
osimModelTemplate = strrep(osimModelTemplate,'L5MASS',sprintf('%.10f',subjectParams.L5Mass));
osimModelTemplate = strrep(osimModelTemplate,'L5INERTIAIXX',sprintf('%.10f',subjectParams.L5Ixx));
osimModelTemplate = strrep(osimModelTemplate,'L5INERTIAIYY',sprintf('%.10f',subjectParams.L5Iyy));
osimModelTemplate = strrep(osimModelTemplate,'L5INERTIAIZZ',sprintf('%.10f',subjectParams.L5Izz));
% L3
osimModelTemplate = strrep(osimModelTemplate,'L3HEIGHT',sprintf('%.10f',subjectParams.L3Height));
osimModelTemplate = strrep(osimModelTemplate,'L3RADIUSX',sprintf('%.10f',subjectParams.L3RadiusX));
osimModelTemplate = strrep(osimModelTemplate,'L3DIAMETERX',sprintf('%.10f',subjectParams.L3DiameterX));
osimModelTemplate = strrep(osimModelTemplate,'L3RADIUSY',sprintf('%.10f',subjectParams.L3RadiusY));
osimModelTemplate = strrep(osimModelTemplate,'L3DIAMETERY',sprintf('%.10f',subjectParams.L3DiameterY));
osimModelTemplate = strrep(osimModelTemplate,'L3HALFHEIGHT',sprintf('%.10f',subjectParams.L3HalfHeight));
osimModelTemplate = strrep(osimModelTemplate,'L3MASS',sprintf('%.10f',subjectParams.L3Mass));
osimModelTemplate = strrep(osimModelTemplate,'L3INERTIAIXX',sprintf('%.10f',subjectParams.L3Ixx));
osimModelTemplate = strrep(osimModelTemplate,'L3INERTIAIYY',sprintf('%.10f',subjectParams.L3Iyy));
osimModelTemplate = strrep(osimModelTemplate,'L3INERTIAIZZ',sprintf('%.10f',subjectParams.L3Izz));
% T12
osimModelTemplate = strrep(osimModelTemplate,'T12HEIGHT',sprintf('%.10f',subjectParams.T12Height));
osimModelTemplate = strrep(osimModelTemplate,'T12RADIUSX',sprintf('%.10f',subjectParams.T12RadiusX));
osimModelTemplate = strrep(osimModelTemplate,'T12DIAMETERX',sprintf('%.10f',subjectParams.T12DiameterX));
osimModelTemplate = strrep(osimModelTemplate,'T12RADIUSY',sprintf('%.10f',subjectParams.T12RadiusY));
osimModelTemplate = strrep(osimModelTemplate,'T12DIAMETERY',sprintf('%.10f',subjectParams.T12DiameterY));
osimModelTemplate = strrep(osimModelTemplate,'T12HALFHEIGHT',sprintf('%.10f',subjectParams.T12HalfHeight));
osimModelTemplate = strrep(osimModelTemplate,'T12MASS',sprintf('%.10f',subjectParams.T12Mass));
osimModelTemplate = strrep(osimModelTemplate,'T12INERTIAIXX',sprintf('%.10f',subjectParams.T12Ixx));
osimModelTemplate = strrep(osimModelTemplate,'T12INERTIAIYY',sprintf('%.10f',subjectParams.T12Iyy));
osimModelTemplate = strrep(osimModelTemplate,'T12INERTIAIZZ',sprintf('%.10f',subjectParams.T12Izz));
% T8
osimModelTemplate = strrep(osimModelTemplate,'T8ALFA',sprintf('%.10f',subjectParams.T8Alfa));
osimModelTemplate = strrep(osimModelTemplate,'T8BETA',sprintf('%.10f',subjectParams.T8Beta));
osimModelTemplate = strrep(osimModelTemplate,'T8GAMMA',sprintf('%.10f',subjectParams.T8Gamma));
osimModelTemplate = strrep(osimModelTemplate,'T8HALFBETA',sprintf('%.10f',subjectParams.T8HalfBeta));
osimModelTemplate = strrep(osimModelTemplate,'T8HALFALFA',sprintf('%.10f',subjectParams.T8HalfAlfa));
osimModelTemplate = strrep(osimModelTemplate,'T8MASS',sprintf('%.10f',subjectParams.T8Mass));
osimModelTemplate = strrep(osimModelTemplate,'T8INERTIAIXX',sprintf('%.10f',subjectParams.T8Ixx));
osimModelTemplate = strrep(osimModelTemplate,'T8INERTIAIYY',sprintf('%.10f',subjectParams.T8Iyy));
osimModelTemplate = strrep(osimModelTemplate,'T8INERTIAIZZ',sprintf('%.10f',subjectParams.T8Izz));
% SHOULDER
osimModelTemplate = strrep(osimModelTemplate,'SHOULDERHEIGHT',sprintf('%.10f',subjectParams.shoulderHeight));
osimModelTemplate = strrep(osimModelTemplate,'SHOULDERRADIUS',sprintf('%.10f',subjectParams.shoulderRadius));
osimModelTemplate = strrep(osimModelTemplate,'SHOULDERDIAMETER',sprintf('%.10f',subjectParams.shoulderDiameter));
osimModelTemplate = strrep(osimModelTemplate,'SHOULDERHALFHEIGHT',sprintf('%.10f',subjectParams.shoulderHalfHeight));
osimModelTemplate = strrep(osimModelTemplate,'SHOULDERPOSZ',sprintf('%.10f',subjectParams.shoulderPosZ));
osimModelTemplate = strrep(osimModelTemplate,'SHOULDERURDFPOSZ',sprintf('%.10f',subjectParams.shoulderUrdfPosZ));
osimModelTemplate = strrep(osimModelTemplate,'SHOULDERMASS',sprintf('%.10f',subjectParams.shoulderMass));
osimModelTemplate = strrep(osimModelTemplate,'SHOULDERINERTIAIXX',sprintf('%.10f',subjectParams.shoulderIxx));
osimModelTemplate = strrep(osimModelTemplate,'SHOULDERINERTIAIYY',sprintf('%.10f',subjectParams.shoulderIyy));
osimModelTemplate = strrep(osimModelTemplate,'SHOULDERINERTIAIZZ',sprintf('%.10f',subjectParams.shoulderIzz));
% UPPER ARM
osimModelTemplate = strrep(osimModelTemplate,'UPPERARMRADIUS',sprintf('%.10f',subjectParams.upperArmRadius));
osimModelTemplate = strrep(osimModelTemplate,'UPPERARMDIAMETER',sprintf('%.10f',subjectParams.upperArmDiameter));
osimModelTemplate = strrep(osimModelTemplate,'UPPERARMHEIGHT',sprintf('%.10f',subjectParams.upperArmHeight));
osimModelTemplate = strrep(osimModelTemplate,'UPPERARMHALFHEIGHT',sprintf('%.10f',subjectParams.upperArmHalfHeight));
osimModelTemplate = strrep(osimModelTemplate,'ARMPOSZ',sprintf('%.10f',subjectParams.armPosZ));
osimModelTemplate = strrep(osimModelTemplate,'ARMURDFPOSZ',sprintf('%.10f',subjectParams.armUrdfPosZ));
osimModelTemplate = strrep(osimModelTemplate,'UPPERARMMASS',sprintf('%.10f',subjectParams.upperArmMass));
osimModelTemplate = strrep(osimModelTemplate,'UPPERARMINERTIAIXX',sprintf('%.10f',subjectParams.upperArmIxx));
osimModelTemplate = strrep(osimModelTemplate,'UPPERARMINERTIAIYY',sprintf('%.10f',subjectParams.upperArmIyy));
osimModelTemplate = strrep(osimModelTemplate,'UPPERARMINERTIAIZZ',sprintf('%.10f',subjectParams.upperArmIzz));
% FORE ARM
osimModelTemplate = strrep(osimModelTemplate,'FOREARMRADIUS',sprintf('%.10f',subjectParams.foreArmRadius));
osimModelTemplate = strrep(osimModelTemplate,'FOREARMDIAMETER',sprintf('%.10f',subjectParams.foreArmDiameter));
osimModelTemplate = strrep(osimModelTemplate,'FOREARMHEIGHT',sprintf('%.10f',subjectParams.foreArmHeight));
osimModelTemplate = strrep(osimModelTemplate,'FOREARMHALFHEIGHT',sprintf('%.10f',subjectParams.foreArmHalfHeight));
osimModelTemplate = strrep(osimModelTemplate,'FOREARMMASS',sprintf('%.10f',subjectParams.foreArmMass));
osimModelTemplate = strrep(osimModelTemplate,'FOREARMINERTIAIXX',sprintf('%.10f',subjectParams.foreArmIxx));
osimModelTemplate = strrep(osimModelTemplate,'FOREARMINERTIAIYY',sprintf('%.10f',subjectParams.foreArmIyy));
osimModelTemplate = strrep(osimModelTemplate,'FOREARMINERTIAIZZ',sprintf('%.10f',subjectParams.foreArmIzz));
% HAND
osimModelTemplate = strrep(osimModelTemplate,'HANDALFA',sprintf('%.10f',subjectParams.handAlfa));
osimModelTemplate = strrep(osimModelTemplate,'HANDBETA',sprintf('%.10f',subjectParams.handBeta));
osimModelTemplate = strrep(osimModelTemplate,'HANDGAMMA',sprintf('%.10f',subjectParams.handGamma));
osimModelTemplate = strrep(osimModelTemplate,'HANDHALFBETA',sprintf('%.10f',subjectParams.handHalfBeta));
osimModelTemplate = strrep(osimModelTemplate,'HANDMASS',sprintf('%.10f',subjectParams.handMass));
osimModelTemplate = strrep(osimModelTemplate,'HANDINERTIAIXX',sprintf('%.10f',subjectParams.handIxx));
osimModelTemplate = strrep(osimModelTemplate,'HANDINERTIAIYY',sprintf('%.10f',subjectParams.handIyy));
osimModelTemplate = strrep(osimModelTemplate,'HANDINERTIAIZZ',sprintf('%.10f',subjectParams.handIzz));
% HEAD
osimModelTemplate = strrep(osimModelTemplate,'HEADRADIUS',sprintf('%.10f',subjectParams.headRadius));
osimModelTemplate = strrep(osimModelTemplate,'HEADDIAMETER',sprintf('%.10f',subjectParams.headDiameter));
osimModelTemplate = strrep(osimModelTemplate,'HEADMASS',sprintf('%.10f',subjectParams.headMass));
osimModelTemplate = strrep(osimModelTemplate,'HEADINERTIAIXX',sprintf('%.10f',subjectParams.headIxx));
osimModelTemplate = strrep(osimModelTemplate,'HEADINERTIAIYY',sprintf('%.10f',subjectParams.headIyy));
osimModelTemplate = strrep(osimModelTemplate,'HEADINERTIAIZZ',sprintf('%.10f',subjectParams.headIzz));
% NECK
osimModelTemplate = strrep(osimModelTemplate,'NECKRADIUS',sprintf('%.10f',subjectParams.neckRadius));
osimModelTemplate = strrep(osimModelTemplate,'NECKDIAMETER',sprintf('%.10f',subjectParams.neckDiameter));
osimModelTemplate = strrep(osimModelTemplate,'NECKHEIGHT',sprintf('%.10f',subjectParams.neckHeight));
osimModelTemplate = strrep(osimModelTemplate,'NECKHALFHEIGHT',sprintf('%.10f',subjectParams.neckHalfHeight));
osimModelTemplate = strrep(osimModelTemplate,'NECKMASS',sprintf('%.10f',subjectParams.neckMass));
osimModelTemplate = strrep(osimModelTemplate,'NECKINERTIAIXX',sprintf('%.10f',subjectParams.neckIxx));
osimModelTemplate = strrep(osimModelTemplate,'NECKINERTIAIYY',sprintf('%.10f',subjectParams.neckIyy));
osimModelTemplate = strrep(osimModelTemplate,'NECKINERTIAIZZ',sprintf('%.10f',subjectParams.neckIzz));

%% POINTS
% PELVIS
[points, ~] = pointsFromName(suit.links{1,1}.points, 'pHipOrigin');
points = subjectParams.hipOrigin_H_pelvis * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PHIPORIGINX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PHIPORIGINY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PHIPORIGINZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{1,1}.points, 'pRightASI');
points = subjectParams.hipOrigin_H_pelvis * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTASIX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTASIY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTASIZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{1,1}.points, 'pLeftASI');
points = subjectParams.hipOrigin_H_pelvis * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTASIX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTASIY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTASIZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{1,1}.points, 'pRightCSI');
points = subjectParams.hipOrigin_H_pelvis * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTCSIX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTCSIY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTCSIZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{1,1}.points, 'pLeftCSI');
points = subjectParams.hipOrigin_H_pelvis * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTCSIX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTCSIY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTCSIZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{1,1}.points, 'pRightIschialTub');
points = subjectParams.hipOrigin_H_pelvis * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTISCHIALTUBX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTISCHIALTUBY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTISCHIALTUBZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{1,1}.points, 'pLeftIschialTub');
points = subjectParams.hipOrigin_H_pelvis * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTISCHIALTUBX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTISCHIALTUBY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTISCHIALTUBZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{1,1}.points, 'pSacrum');
points = subjectParams.hipOrigin_H_pelvis * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PSACRUMX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PSACRUMY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PSACRUMZ',sprintf('%.10f',points(3)));
% L5
[points, ~] = pointsFromName(suit.links{2,1}.points, 'pL5SpinalProcess');
points = subjectParams.jL5S1_H_l5 * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PL5SPINALPROCESSX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PL5SPINALPROCESSY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PL5SPINALPROCESSZ',sprintf('%.10f',points(3)));
% L3
[points, ~] = pointsFromName(suit.links{3,1}.points, 'pL3SpinalProcess');
points = subjectParams.jL4L3_H_l3 * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PL3SPINALPROCESSX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PL3SPINALPROCESSY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PL3SPINALPROCESSZ',sprintf('%.10f',points(3)));
% T12
[points, ~] = pointsFromName(suit.links{4,1}.points, 'pT12SpinalProcess');
points = subjectParams.jL1T12_H_T12 * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PT12SPINALPROCESSX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PT12SPINALPROCESSY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PT12SPINALPROCESSZ',sprintf('%.10f',points(3)));
% T8
[points, ~] = pointsFromName(suit.links{5,1}.points, 'pPX');
points = subjectParams.jT9T8_H_T8 * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PPXX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PPXY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PPXZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{5,1}.points, 'pIJ');
points = subjectParams.jT9T8_H_T8 * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PIJX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PIJY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PIJZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{5,1}.points, 'pT4SpinalProcess');
points = subjectParams.jT9T8_H_T8 * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PT4SPINALPROCESSX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PT4SPINALPROCESSY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PT4SPINALPROCESSZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{5,1}.points, 'pT8SpinalProcess');
points = subjectParams.jT9T8_H_T8 * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PT8SPINALPROCESSX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PT8SPINALPROCESSY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PT8SPINALPROCESSZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{5,1}.points, 'pC7SpinalProcess');
points = subjectParams.jT9T8_H_T8 * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PC7SPINALPROCESSX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PC7SPINALPROCESSY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PC7SPINALPROCESSZ',sprintf('%.10f',points(3)));
% HEAD
[points, ~] = pointsFromName(suit.links{7,1}.points, 'pTopOfHead');
points = subjectParams.jC1Head_H_head * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PTOPOFHEADX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PTOPOFHEADY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PTOPOFHEADZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{7,1}.points, 'pRightAuricularis');
points = subjectParams.jC1Head_H_head * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTAURICULARISX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTAURICULARISY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTAURICULARISZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{7,1}.points, 'pLeftAuricularis');
points = subjectParams.jC1Head_H_head * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTAURICULARISX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTAURICULARISY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTAURICULARISZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{7,1}.points, 'pBackOfHead');
points = subjectParams.jC1Head_H_head * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PBACKOFHEADX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PBACKOFHEADY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PBACKOFHEADZ',sprintf('%.10f',points(3)));
% RIGHT SHOULDER
[points, ~] = pointsFromName(suit.links{8,1}.points, 'pRightAcromion');
points = subjectParams.jRightC7Shoulder_H_rightShoulder * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTACROMIONX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTACROMIONY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTACROMIONZ',sprintf('%.10f',points(3)));
% LEFT SHOULDER
[points, ~] = pointsFromName(suit.links{12,1}.points, 'pLeftAcromion');
points = subjectParams.jLeftC7Shoulder_H_leftShoulder * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTACROMIONX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTACROMIONY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTACROMIONZ',sprintf('%.10f',points(3)));
% RIGHT UPPER ARM
[points, ~] = pointsFromName(suit.links{9,1}.points, 'pRightArmLatEpicondyle');
points = subjectParams.jRightShoulder_H_rightUpperArm * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTARMLATEPICONDYLEX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTARMLATEPICONDYLEY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTARMLATEPICONDYLEZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{9,1}.points, 'pRightArmMedEpicondyle');
points = subjectParams.jRightShoulder_H_rightUpperArm * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTARMMEDEPICONDYLEX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTARMMEDEPICONDYLEY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTARMMEDEPICONDYLEZ',sprintf('%.10f',points(3)));
% LEFT UPPER ARM
[points, ~] = pointsFromName(suit.links{13,1}.points, 'pLeftArmLatEpicondyle');
points = subjectParams.jLeftShoulder_H_leftUpperArm * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTARMLATEPICONDYLEX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTARMLATEPICONDYLEY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTARMLATEPICONDYLEZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{13,1}.points, 'pLeftArmMedEpicondyle');
points = subjectParams.jLeftShoulder_H_leftUpperArm * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTARMMEDEPICONDYLEX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTARMMEDEPICONDYLEY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTARMMEDEPICONDYLEZ',sprintf('%.10f',points(3)));
% RIGHT FORE ARM
[points, ~] = pointsFromName(suit.links{10,1}.points, 'pRightUlnarStyloid');
points = subjectParams.jRightElbow_H_rightForeArm * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTULNARSTYLOIDX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTULNARSTYLOIDY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTULNARSTYLOIDZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{10,1}.points, 'pRightRadialStyloid');
points = subjectParams.jRightElbow_H_rightForeArm * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTRADIALSTYLOIDX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTRADIALSTYLOIDY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTRADIALSTYLOIDZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{10,1}.points, 'pRightOlecranon');
points = subjectParams.jRightElbow_H_rightForeArm * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTOLECRANONX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTOLECRANONY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTOLECRANONZ',sprintf('%.10f',points(3)));
% LEFT FORE ARM
[points, ~] = pointsFromName(suit.links{14,1}.points, 'pLeftUlnarStyloid');
points = subjectParams.jLeftElbow_H_leftForeArm * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTULNARSTYLOIDX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTULNARSTYLOIDY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTULNARSTYLOIDZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{14,1}.points, 'pLeftRadialStyloid');
points = subjectParams.jLeftElbow_H_leftForeArm * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTRADIALSTYLOIDX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTRADIALSTYLOIDY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTRADIALSTYLOIDZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{14,1}.points, 'pLeftOlecranon');
points = subjectParams.jLeftElbow_H_leftForeArm * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTOLECRANONX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTOLECRANONY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTOLECRANONZ',sprintf('%.10f',points(3)));
% RIGHT HAND
[points, ~] = pointsFromName(suit.links{11,1}.points, 'pRightTopOfHand');
points = subjectParams.jRightWrist_H_rightHand * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTTOPOFHANDX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTTOPOFHANDY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTTOPOFHANDZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{11,1}.points, 'pRightPinky');
points = subjectParams.jRightWrist_H_rightHand * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTPINKYX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTPINKYY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTPINKYZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{11,1}.points, 'pRightBallHand');
points = subjectParams.jRightWrist_H_rightHand * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTBALLHANDX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTBALLHANDY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTBALLHANDZ',sprintf('%.10f',points(3)));
% LEFT HAND
[points, ~] = pointsFromName(suit.links{15,1}.points, 'pLeftTopOfHand');
points = subjectParams.jLeftWrist_H_leftHand * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTTOPOFHANDX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTTOPOFHANDY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTTOPOFHANDZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{15,1}.points, 'pLeftPinky');
points = subjectParams.jLeftWrist_H_leftHand * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTPINKYX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTPINKYY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTPINKYZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{15,1}.points, 'pLeftBallHand');
points = subjectParams.jLeftWrist_H_leftHand * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTBALLHANDX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTBALLHANDY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTBALLHANDZ',sprintf('%.10f',points(3)));
% RIGHT UPPER LEG
[points, ~] = pointsFromName(suit.links{16,1}.points, 'pRightGreaterTrochanter');
points = subjectParams.hip_H_upperLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTGREATERTROCHANTERX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTGREATERTROCHANTERY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTGREATERTROCHANTERZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{16,1}.points, 'pRightPatella');
points = subjectParams.hip_H_upperLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTPATELLAX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTPATELLAY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTPATELLAZ',sprintf('%.10f',points(3)));
% RIGHT LOWER LEG
[points, ~] = pointsFromName(suit.links{17,1}.points, 'pRightKneeLatEpicondyle');
points = subjectParams.knee_H_lowerLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTKNEELATEPICONDYLEX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTKNEELATEPICONDYLEY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTKNEELATEPICONDYLEZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{17,1}.points, 'pRightKneeMedEpicondyle');
points = subjectParams.knee_H_lowerLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTKNEEMEDEPICONDYLEX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTKNEEMEDEPICONDYLEY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTKNEEMEDEPICONDYLEZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{17,1}.points, 'pRightLatMalleolus');
points = subjectParams.knee_H_lowerLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTLATMALLEOLUSX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTLATMALLEOLUSY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTLATMALLEOLUSZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{17,1}.points, 'pRightMedMalleolus');
points = subjectParams.knee_H_lowerLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTMEDMALLEOLUSX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTMEDMALLEOLUSY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTMEDMALLEOLUSZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{17,1}.points, 'pRightTibialTub');
points = subjectParams.knee_H_lowerLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTTIBIALTUBX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTTIBIALTUBY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTTIBIALTUBZ',sprintf('%.10f',points(3)));
% LEFT UPPER LEG
[points, ~] = pointsFromName(suit.links{20,1}.points, 'pLeftGreaterTrochanter');
points = subjectParams.hip_H_upperLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTGREATERTROCHANTERX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTGREATERTROCHANTERY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTGREATERTROCHANTERZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{20,1}.points, 'pLeftPatella');
points = subjectParams.hip_H_upperLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTPATELLAX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTPATELLAY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTPATELLAZ',sprintf('%.10f',points(3)));
% LEFT LOWER LEG
[points, ~] = pointsFromName(suit.links{21,1}.points, 'pLeftKneeLatEpicondyle');
points = subjectParams.knee_H_lowerLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTKNEELATEPICONDYLEX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTKNEELATEPICONDYLEY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTKNEELATEPICONDYLEZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{21,1}.points, 'pLeftKneeMedEpicondyle');
points = subjectParams.knee_H_lowerLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTKNEEMEDEPICONDYLEX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTKNEEMEDEPICONDYLEY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTKNEEMEDEPICONDYLEZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{21,1}.points, 'pLeftLatMalleolus');
points = subjectParams.knee_H_lowerLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTLATMALLEOLUSX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTLATMALLEOLUSY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTLATMALLEOLUSZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{21,1}.points, 'pLeftMedMalleolus');
points = subjectParams.knee_H_lowerLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTMEDMALLEOLUSX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTMEDMALLEOLUSY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTMEDMALLEOLUSZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{21,1}.points, 'pLeftTibialTub');
points = subjectParams.knee_H_lowerLeg * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTTIBIALTUBX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTTIBIALTUBY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTTIBIALTUBZ',sprintf('%.10f',points(3)));
% RIGHT FOOT
[points, ~] = pointsFromName(suit.links{18,1}.points, 'pRightHeelFoot');
points = subjectParams.ankle_H_foot * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTHEELFOOTX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTHEELFOOTY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTHEELFOOTZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{18,1}.points, 'pRightFirstMetatarsal');
points = subjectParams.ankle_H_foot * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTFIRSTMETATARSALX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTFIRSTMETATARSALY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTFIRSTMETATARSALZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{18,1}.points, 'pRightFifthMetatarsal');
points = subjectParams.ankle_H_foot * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTFIFTHMETATARSALX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTFIFTHMETATARSALY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTFIFTHMETATARSALZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{18,1}.points, 'pRightPivotFoot');
points = subjectParams.ankle_H_foot * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTPIVOTFOOTX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTPIVOTFOOTY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTPIVOTFOOTZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{18,1}.points, 'pRightHeelCenter');
points = subjectParams.ankle_H_foot * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTHEELCENTERX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTHEELCENTERY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTHEELCENTERZ',sprintf('%.10f',points(3)));
% RIGHT TOE
[points, ~] = pointsFromName(suit.links{19,1}.points, 'pRightToe');
points = subjectParams.ballFoot_H_toe * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTTOEX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTTOEY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PRIGHTTOEZ',sprintf('%.10f',points(3)));
% LEFT FOOT
[points, ~] = pointsFromName(suit.links{22,1}.points, 'pLeftHeelFoot');
points = subjectParams.ankle_H_foot * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTHEELFOOTX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTHEELFOOTY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTHEELFOOTZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{22,1}.points, 'pLeftFirstMetatarsal');
points = subjectParams.ankle_H_foot * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTFIRSTMETATARSALX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTFIRSTMETATARSALY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTFIRSTMETATARSALZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{22,1}.points, 'pLeftFifthMetatarsal');
points = subjectParams.ankle_H_foot * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTFIFTHMETATARSALX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTFIFTHMETATARSALY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTFIFTHMETATARSALZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{22,1}.points, 'pLeftPivotFoot');
points = subjectParams.ankle_H_foot * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTPIVOTFOOTX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTPIVOTFOOTY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTPIVOTFOOTZ',sprintf('%.10f',points(3)));
[points, ~] = pointsFromName(suit.links{22,1}.points, 'pLeftHeelCenter');
points = subjectParams.ankle_H_foot * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTHEELCENTERX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTHEELCENTERY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTHEELCENTERZ',sprintf('%.10f',points(3)));
% LEFT TOE
[points, ~] = pointsFromName(suit.links{23,1}.points, 'pLeftToe');
points = subjectParams.ballFoot_H_toe * [points;1];
osimModelTemplate = strrep(osimModelTemplate,'PLEFTTOEX',sprintf('%.10f',points(1)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTTOEY',sprintf('%.10f',points(2)));
osimModelTemplate = strrep(osimModelTemplate,'PLEFTTOEZ',sprintf('%.10f',points(3)));

if nargin == 3
    [dir,~,~] = fileparts(filename);
    if ~exist(dir,'dir')
        mkdir(dir);
    end
fileID = fopen(filename,'w');
fprintf(fileID,'%s', osimModelTemplate);
fclose(fileID);
end
end

function [points, found] = pointsFromName(pointsStruct, pointName)
%POINTSFROMNAME
% Inputs:
% points    : struct of type points;
% pointName : string denoting the point you are looking for;
% Outputs:
% points : 3x1 vector of points;
% found  : true if point has been found, false otherwise.

for indx = 1 : pointsStruct.nrOfPoints
    if  strcmp(pointsStruct.label{1,indx},pointName)
        points = pointsStruct.pointsValue(:,indx);
        found = true;
        break;
    else 
        found = false;
    end   
end
if found == false
    error(sprintf('Something wrong in the acquisition! Point label <%s> not found.',pointName));
end
end
