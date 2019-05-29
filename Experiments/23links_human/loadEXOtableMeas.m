%% Load and extract data from EXO table
EXO.dataFilename  = fullfile(bucket.datasetRoot, 'ForceDataTable.csv');
EXO.extractedDataRaw = table2array(readtable(EXO.dataFilename,'Delimiter',';'));

% Generic raw info table (labels/extraction range per label)
EXO.tableLabels = {'shoulderAngle';
    'F_arm_scher';
    'F_arm_support';
    'F_ASkraft_x';
    'F_ASkraft_y';
    'F_KGkraft_x';
    'F_KGkraft_y';
    'M_support'};

for labelIdx = 1 : size(EXO.tableLabels,1)
    EXO.tableInfo(labelIdx).labels = EXO.tableLabels{labelIdx};
    EXO.tableInfo(labelIdx).range  = (labelIdx:8:96);
end

EXO.extractedTable(subjectID).shoulder_angles  = EXO.extractedDataRaw(:,EXO.tableInfo(1).range(subjectID));
EXO.extractedTable(subjectID).F_arm_scher   = EXO.extractedDataRaw(:,EXO.tableInfo(2).range(subjectID));
EXO.extractedTable(subjectID).F_arm_support = EXO.extractedDataRaw(:,EXO.tableInfo(3).range(subjectID));
EXO.extractedTable(subjectID).F_ASkraft_x   = EXO.extractedDataRaw(:,EXO.tableInfo(4).range(subjectID));
EXO.extractedTable(subjectID).F_ASkraft_y   = EXO.extractedDataRaw(:,EXO.tableInfo(5).range(subjectID));
EXO.extractedTable(subjectID).F_KGkraft_x   = EXO.extractedDataRaw(:,EXO.tableInfo(6).range(subjectID));
EXO.extractedTable(subjectID).F_KGkraft_y   = EXO.extractedDataRaw(:,EXO.tableInfo(7).range(subjectID));

% Correction: Computation of a
% newTableTorque = F_arm_support * 0.6895 * subject upper arm length
% Note: The point where the exo applies the force at the upper arm is
%       0.6895 of the total upper arm length (i.e., the application point
%       of the force does not coincide with the elbow joint!!!).
EXO.extractedTable(subjectID).M_support_mod    = EXO.extractedDataRaw(:,EXO.tableInfo(8).range(subjectID)) * ...
    0.6895 * subjectParamsFromData.leftUpperArm_y;
