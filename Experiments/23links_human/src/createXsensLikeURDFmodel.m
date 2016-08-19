function [urdfModelTemplate] = createXsensLikeURDFmodel(subjectParams)
%CREATEXSENSLIKEURDFMODEL generates a URDF model of the subject.

urdfModelTemplate = fileread('XSensModelStyle_URDFtemplate.urdf');
    
% FOOT
urdfModelTemplate = strrep(urdfModelTemplate,'FOOTALFA',num2str(subjectParams.footAlfa));
urdfModelTemplate = strrep(urdfModelTemplate,'FOOTBETA',num2str(subjectParams.footBeta));
urdfModelTemplate = strrep(urdfModelTemplate,'FOOTGAMMA',num2str(subjectParams.footGamma));
urdfModelTemplate = strrep(urdfModelTemplate,'FOOTHALFBETA',num2str(subjectParams.footHalfBeta));
urdfModelTemplate = strrep(urdfModelTemplate,'FOOTHALFGAMMA',num2str(subjectParams.footHalfGamma));
urdfModelTemplate = strrep(urdfModelTemplate,'ANKLEPOSX',num2str(subjectParams.anklePosX));
urdfModelTemplate = strrep(urdfModelTemplate,'TOEPOSX',num2str(subjectParams.toePosX));
urdfModelTemplate = strrep(urdfModelTemplate,'FOOTMASS',num2str(subjectParams.footMass));
urdfModelTemplate = strrep(urdfModelTemplate,'FOOTINERTIAIXX',num2str(subjectParams.footIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'FOOTINERTIAIYY',num2str(subjectParams.footIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'FOOTINERTIAIZZ',num2str(subjectParams.footIzz));
% TOE
urdfModelTemplate = strrep(urdfModelTemplate,'TOEALFA',num2str(subjectParams.toeAlfa));
urdfModelTemplate = strrep(urdfModelTemplate,'TOEBETA',num2str(subjectParams.toeBeta));
urdfModelTemplate = strrep(urdfModelTemplate,'TOEGAMMA',num2str(subjectParams.toeGamma));
urdfModelTemplate = strrep(urdfModelTemplate,'TOEHALFGAMMA',num2str(subjectParams.toeHalfGamma));
urdfModelTemplate = strrep(urdfModelTemplate,'TOEMASS',num2str(subjectParams.toeMass));
urdfModelTemplate = strrep(urdfModelTemplate,'TOEINERTIAIXX',num2str(subjectParams.toeIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'TOEINERTIAIYY',num2str(subjectParams.toeIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'TOEINERTIAIZZ',num2str(subjectParams.toeIzz));
% LOWER LEG
urdfModelTemplate = strrep(urdfModelTemplate,'LOWERLEGHEIGHT',num2str(subjectParams.lowerLegHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'LOWERLEGRADIUS',num2str(subjectParams.lowerLegRadius));
urdfModelTemplate = strrep(urdfModelTemplate,'LOWERLEGHALFHEIGHT',num2str(subjectParams.lowerLegHalfHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'LOWERLEGDIAMETER',num2str(subjectParams.lowerLegDiameter));
urdfModelTemplate = strrep(urdfModelTemplate,'LOWERLEGMASS',num2str(subjectParams.lowerLegMass));
urdfModelTemplate = strrep(urdfModelTemplate,'LOWERLEGINERTIAIXX',num2str(subjectParams.lowerLegIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'LOWERLEGINERTIAIYY',num2str(subjectParams.lowerLegIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'LOWERLEGINERTIAIZZ',num2str(subjectParams.lowerLegIzz));
% UPPER LEG
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERLEGHEIGHT',num2str(subjectParams.upperLegHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERLEGRADIUS',num2str(subjectParams.upperLegRadius));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERLEGDIAMETER',num2str(subjectParams.upperLegDiameter));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERLEGHALFHEIGHT',num2str(subjectParams.upperLegHalfHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'HIPPOSY',num2str(subjectParams.HipPosY));
urdfModelTemplate = strrep(urdfModelTemplate,'HIPDOUBLEPOSY',num2str(subjectParams.HipDoublePosY));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERLEGMASS',num2str(subjectParams.upperLegMass));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERLEGINERTIAIXX',num2str(subjectParams.upperLegIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERLEGINERTIAIYY',num2str(subjectParams.upperLegIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERLEGINERTIAIZZ',num2str(subjectParams.upperLegIzz));
% TORSO
urdfModelTemplate = strrep(urdfModelTemplate,'TORSOALFA',num2str(subjectParams.torsoAlfa));
urdfModelTemplate = strrep(urdfModelTemplate,'TORSOBETA',num2str(subjectParams.torsoBeta));
urdfModelTemplate = strrep(urdfModelTemplate,'TORSOGAMMA',num2str(subjectParams.torsoGamma));
urdfModelTemplate = strrep(urdfModelTemplate,'TORSOHALFBETA',num2str(subjectParams.torsoHalfBeta));
urdfModelTemplate = strrep(urdfModelTemplate,'TORSOMASS',num2str(subjectParams.torsoMass));
urdfModelTemplate = strrep(urdfModelTemplate,'TORSOINERTIAIXX',num2str(subjectParams.torsoIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'TORSOINERTIAIYY',num2str(subjectParams.torsoIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'TORSOINERTIAIZZ',num2str(subjectParams.torsoIzz));
% PELVIS
urdfModelTemplate = strrep(urdfModelTemplate,'PELVISALFA',num2str(subjectParams.pelvisAlfa));
urdfModelTemplate = strrep(urdfModelTemplate,'PELVISBETA',num2str(subjectParams.pelvisBeta));
urdfModelTemplate = strrep(urdfModelTemplate,'PELVISGAMMA',num2str(subjectParams.pelvisGamma));
urdfModelTemplate = strrep(urdfModelTemplate,'PELVISHALFBETA',num2str(subjectParams.pelvisHalfBeta));
urdfModelTemplate = strrep(urdfModelTemplate,'PELVISMASS',num2str(subjectParams.pelvisMass));
urdfModelTemplate = strrep(urdfModelTemplate,'PELVISINERTIAIXX',num2str(subjectParams.pelvisIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'PELVISINERTIAIYY',num2str(subjectParams.pelvisIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'PELVISINERTIAIZZ',num2str(subjectParams.pelvisIzz));
% L5
urdfModelTemplate = strrep(urdfModelTemplate,'L5HEIGHT',num2str(subjectParams.L5Height));
urdfModelTemplate = strrep(urdfModelTemplate,'L5RADIUSX',num2str(subjectParams.L5RadiusX));
urdfModelTemplate = strrep(urdfModelTemplate,'L5DIAMETERX',num2str(subjectParams.L5DiameterX));
urdfModelTemplate = strrep(urdfModelTemplate,'L5RADIUSY',num2str(subjectParams.L5RadiusY));
urdfModelTemplate = strrep(urdfModelTemplate,'L5DIAMETERY',num2str(subjectParams.L5DiameterY));
urdfModelTemplate = strrep(urdfModelTemplate,'L5HALFHEIGHT',num2str(subjectParams.L5HalfHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'L5MASS',num2str(subjectParams.L5Mass));
urdfModelTemplate = strrep(urdfModelTemplate,'L5INERTIAIXX',num2str(subjectParams.L5Ixx));
urdfModelTemplate = strrep(urdfModelTemplate,'L5INERTIAIYY',num2str(subjectParams.L5Iyy));
urdfModelTemplate = strrep(urdfModelTemplate,'L5INERTIAIZZ',num2str(subjectParams.L5Izz));
% L3
urdfModelTemplate = strrep(urdfModelTemplate,'L3HEIGHT',num2str(subjectParams.L3Height));
urdfModelTemplate = strrep(urdfModelTemplate,'L3RADIUSX',num2str(subjectParams.L3RadiusX));
urdfModelTemplate = strrep(urdfModelTemplate,'L3DIAMETERX',num2str(subjectParams.L3DiameterX));
urdfModelTemplate = strrep(urdfModelTemplate,'L3RADIUSY',num2str(subjectParams.L3RadiusY));
urdfModelTemplate = strrep(urdfModelTemplate,'L3DIAMETERY',num2str(subjectParams.L3DiameterY));
urdfModelTemplate = strrep(urdfModelTemplate,'L3HALFHEIGHT',num2str(subjectParams.L3HalfHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'L3MASS',num2str(subjectParams.L3Mass));
urdfModelTemplate = strrep(urdfModelTemplate,'L3INERTIAIXX',num2str(subjectParams.L3Ixx));
urdfModelTemplate = strrep(urdfModelTemplate,'L3INERTIAIYY',num2str(subjectParams.L3Iyy));
urdfModelTemplate = strrep(urdfModelTemplate,'L3INERTIAIZZ',num2str(subjectParams.L3Izz));
% T12
urdfModelTemplate = strrep(urdfModelTemplate,'T12HEIGHT',num2str(subjectParams.T12Height));
urdfModelTemplate = strrep(urdfModelTemplate,'T12RADIUSX',num2str(subjectParams.T12RadiusX));
urdfModelTemplate = strrep(urdfModelTemplate,'T12DIAMETERX',num2str(subjectParams.T12DiameterX));
urdfModelTemplate = strrep(urdfModelTemplate,'T12RADIUSY',num2str(subjectParams.T12RadiusY));
urdfModelTemplate = strrep(urdfModelTemplate,'T12DIAMETERY',num2str(subjectParams.T12DiameterY));
urdfModelTemplate = strrep(urdfModelTemplate,'T12HALFHEIGHT',num2str(subjectParams.T12HalfHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'T12MASS',num2str(subjectParams.T12Mass));
urdfModelTemplate = strrep(urdfModelTemplate,'T12INERTIAIXX',num2str(subjectParams.T12Ixx));
urdfModelTemplate = strrep(urdfModelTemplate,'T12INERTIAIYY',num2str(subjectParams.T12Iyy));
urdfModelTemplate = strrep(urdfModelTemplate,'T12INERTIAIZZ',num2str(subjectParams.T12Izz));
% T8
urdfModelTemplate = strrep(urdfModelTemplate,'T8ALFA',num2str(subjectParams.T8Alfa));
urdfModelTemplate = strrep(urdfModelTemplate,'T8BETA',num2str(subjectParams.T8Beta));
urdfModelTemplate = strrep(urdfModelTemplate,'T8GAMMA',num2str(subjectParams.T8Gamma));
urdfModelTemplate = strrep(urdfModelTemplate,'T8HALFBETA',num2str(subjectParams.T8HalfBeta));
urdfModelTemplate = strrep(urdfModelTemplate,'T8HALFALFA',num2str(subjectParams.T8HalfAlfa));
urdfModelTemplate = strrep(urdfModelTemplate,'T8MASS',num2str(subjectParams.T8Mass));
urdfModelTemplate = strrep(urdfModelTemplate,'T8INERTIAIXX',num2str(subjectParams.T8Ixx));
urdfModelTemplate = strrep(urdfModelTemplate,'T8INERTIAIYY',num2str(subjectParams.T8Iyy));
urdfModelTemplate = strrep(urdfModelTemplate,'T8INERTIAIZZ',num2str(subjectParams.T8Izz));
% SHOULDER
urdfModelTemplate = strrep(urdfModelTemplate,'SHOULDERHEIGHT',num2str(subjectParams.shoulderHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'SHOULDERRADIUS',num2str(subjectParams.shoulderRadius));
urdfModelTemplate = strrep(urdfModelTemplate,'SHOULDERDIAMETER',num2str(subjectParams.shoulderDiameter));
urdfModelTemplate = strrep(urdfModelTemplate,'SHOULDERHALFHEIGHT',num2str(subjectParams.shoulderHalfHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'SHOULDERPOSZ',num2str(subjectParams.shoulderPosZ));
urdfModelTemplate = strrep(urdfModelTemplate,'SHOULDERURDFPOSZ',num2str(subjectParams.shoulderUrdfPosZ));
urdfModelTemplate = strrep(urdfModelTemplate,'SHOULDERMASS',num2str(subjectParams.shoulderMass));
urdfModelTemplate = strrep(urdfModelTemplate,'SHOULDERINERTIAIXX',num2str(subjectParams.shoulderIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'SHOULDERINERTIAIYY',num2str(subjectParams.shoulderIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'SHOULDERINERTIAIZZ',num2str(subjectParams.shoulderIzz));
% UPPER ARM
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERARMRADIUS',num2str(subjectParams.upperArmRadius));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERARMDIAMETER',num2str(subjectParams.upperArmDiameter));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERARMHEIGHT',num2str(subjectParams.upperArmHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERARMHALFHEIGHT',num2str(subjectParams.upperArmHalfHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'ARMPOSZ',num2str(subjectParams.armPosZ));
urdfModelTemplate = strrep(urdfModelTemplate,'ARMURDFPOSZ',num2str(subjectParams.armUrdfPosZ));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERARMMASS',num2str(subjectParams.upperArmMass));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERARMINERTIAIXX',num2str(subjectParams.upperArmIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERARMINERTIAIYY',num2str(subjectParams.upperArmIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'UPPERARMINERTIAIZZ',num2str(subjectParams.upperArmIzz));
% FORE ARM
urdfModelTemplate = strrep(urdfModelTemplate,'FOREARMRADIUS',num2str(subjectParams.foreArmRadius));
urdfModelTemplate = strrep(urdfModelTemplate,'FOREARMDIAMETER',num2str(subjectParams.foreArmDiameter));
urdfModelTemplate = strrep(urdfModelTemplate,'FOREARMHEIGHT',num2str(subjectParams.foreArmHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'FOREARMHALFHEIGHT',num2str(subjectParams.foreArmHalfHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'FOREARMMASS',num2str(subjectParams.foreArmMass));
urdfModelTemplate = strrep(urdfModelTemplate,'FOREARMINERTIAIXX',num2str(subjectParams.foreArmIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'FOREARMINERTIAIYY',num2str(subjectParams.foreArmIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'FOREARMINERTIAIZZ',num2str(subjectParams.foreArmIzz));

% HAND
urdfModelTemplate = strrep(urdfModelTemplate,'HANDALFA',num2str(subjectParams.handAlfa));
urdfModelTemplate = strrep(urdfModelTemplate,'HANDBETA',num2str(subjectParams.handBeta));
urdfModelTemplate = strrep(urdfModelTemplate,'HANDGAMMA',num2str(subjectParams.handGamma));
urdfModelTemplate = strrep(urdfModelTemplate,'HANDHALFBETA',num2str(subjectParams.handHalfBeta));
urdfModelTemplate = strrep(urdfModelTemplate,'HANDMASS',num2str(subjectParams.handMass));
urdfModelTemplate = strrep(urdfModelTemplate,'HANDINERTIAIXX',num2str(subjectParams.handIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'HANDINERTIAIYY',num2str(subjectParams.handIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'HANDINERTIAIZZ',num2str(subjectParams.handIzz));
% HEAD
urdfModelTemplate = strrep(urdfModelTemplate,'HEADRADIUS',num2str(subjectParams.headRadius));
urdfModelTemplate = strrep(urdfModelTemplate,'HEADDIAMETER',num2str(subjectParams.headDiameter));
urdfModelTemplate = strrep(urdfModelTemplate,'HEADMASS',num2str(subjectParams.headMass));
urdfModelTemplate = strrep(urdfModelTemplate,'HEADINERTIAIXX',num2str(subjectParams.headIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'HEADINERTIAIYY',num2str(subjectParams.headIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'HEADINERTIAIZZ',num2str(subjectParams.headIzz));
% NECK
urdfModelTemplate = strrep(urdfModelTemplate,'NECKRADIUS',num2str(subjectParams.neckRadius));
urdfModelTemplate = strrep(urdfModelTemplate,'NECKDIAMETER',num2str(subjectParams.neckDiameter));
urdfModelTemplate = strrep(urdfModelTemplate,'NECKHEIGHT',num2str(subjectParams.neckHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'NECKHALFHEIGHT',num2str(subjectParams.neckHalfHeight));
urdfModelTemplate = strrep(urdfModelTemplate,'NECKMASS',num2str(subjectParams.neckMass));
urdfModelTemplate = strrep(urdfModelTemplate,'NECKINERTIAIXX',num2str(subjectParams.neckIxx));
urdfModelTemplate = strrep(urdfModelTemplate,'NECKINERTIAIYY',num2str(subjectParams.neckIyy));
urdfModelTemplate = strrep(urdfModelTemplate,'NECKINERTIAIZZ',num2str(subjectParams.neckIzz));
end
