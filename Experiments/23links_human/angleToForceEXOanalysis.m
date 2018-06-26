
%% Extraction parameters for the EXO angle-to-force characterization
% This script has to be launched after the extraction of the
% subjectParamsFromData from all the dataset subjects!

pathToExoHumanInfo = fullfile(bucket.datasetRoot,'exoHumanInfo');
if ~exist(pathToExoHumanInfo,'dir')
    mkdir(pathToExoHumanInfo);
end

subjectID = [1,2,3,4,5,6,7,8,9,10,11,12];

for subjIdx = 1 : length(subjectID)
    pathToSubject = fullfile(bucket.datasetRoot, sprintf('S%02d',subjectID(subjIdx)));
    % load the proper subjectParamsFromData struct
    load(fullfile(pathToSubject,'subjectParamsFromData.mat'));
    % fill the new struct
    exoHumanInfo(subjIdx).Subject = sprintf('S%02d',subjectID(subjIdx));
    % --> upperArm length/mass symmetric per model construction
    exoHumanInfo(subjIdx).upperArm_length   = subjectParamsFromData.rightUpperArm_y;
    exoHumanInfo(subjIdx).upperArm_mass     = subjectParamsFromData.rightUpperArmMass;
    % --> foreArm mass symmetric per model construction
    exoHumanInfo(subjIdx).foreArm_mass     = subjectParamsFromData.rightForeArmMass;
    % --> hand mass different from model (drill/no drill)
    exoHumanInfo(subjIdx).rightHand_mass_drill   = subjectParamsFromData.rightHandMass;
    exoHumanInfo(subjIdx).leftHand_mass_noDrill = subjectParamsFromData.leftHandMass;
end

writetable(struct2table(exoHumanInfo), 'exoHumanInfo.csv')
copyfile('exoHumanInfo.csv',pathToExoHumanInfo);
