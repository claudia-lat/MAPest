
%% =========================== EXO ANALYSIS ===============================
% Extract data from EXO table and compare with the tauFirst(1,:).
% Note: the exo angles has to be compared with the tauFirst x and not 
% from tau_MAP anymore.

% Load EXO data
EXO.dataFilename = fullfile(bucket.datasetRoot, 'EXOforceData.csv');

% Extract EXO data
EXO.extractedData = table2array(readtable(EXO.dataFilename,'Delimiter',';'));
EXO.extractedData_noHeader = str2double(EXO.extractedData(2:end,:));
EXO.subjTorqueID = (4:3:37);

for blockIdx = 1 : block.nrOfBlocks
    %% -------Right shoulder
    EXO.qToCompare_right = (- CoC(blockIdx).Rsho_qFirst(1,:) + 90)'; % operation to compare the angles: change sign and then +90 deg
    EXO.qToCompare_right_round = round(EXO.qToCompare_right);
    
    EXO.tau_EXO_right = zeros(1,size(EXO.qToCompare_right_round,1));
    for qIdx = 1 : size(EXO.qToCompare_right_round,1)
        for tableIdx = 1 : size(EXO.extractedData_noHeader,1)
            if (EXO.qToCompare_right_round(qIdx) == EXO.extractedData_noHeader(tableIdx,1))
                EXO.tau_EXO_right(qIdx) = - EXO.extractedData_noHeader(tableIdx,EXO.subjTorqueID(subjectID));
            end
        end
    end
    
    %% -------Left shoulder
    EXO.qToCompare_left = (CoC(blockIdx).Lsho_qFirst(1,:) + 90)'; % operation to compare the angles: +90 deg
    EXO.qToCompare_left_round = round(EXO.qToCompare_left);
    
    EXO.tau_EXO_left = zeros(1,size(EXO.qToCompare_left_round,1));
    for qIdx = 1 : size(EXO.qToCompare_left_round,1)
        for tableIdx = 1 : size(EXO.extractedData_noHeader,1)
            if (EXO.qToCompare_left_round(qIdx) == EXO.extractedData_noHeader(tableIdx,1))
                EXO.tau_EXO_left(qIdx) = EXO.extractedData_noHeader(tableIdx,EXO.subjTorqueID(subjectID));
            end
        end
    end
    
    %% Save into a struct
    exo(blockIdx).block = block.labels(blockIdx);
    exo(blockIdx).masterTime = synchroKin(blockIdx).masterTime;
    exo(blockIdx).torqueFromTable_right = EXO.tau_EXO_right;
    exo(blockIdx).torqueFromTable_left  = EXO.tau_EXO_left;
    % tauFirst_MAPest - tau_EXOfromTable
    exo(blockIdx).torqueDiff_right = CoC(blockIdx).Rsho_tauFirst(1,:) - EXO.tau_EXO_right;
    exo(blockIdx).torqueDiff_left  = CoC(blockIdx).Lsho_tauFirst(1,:) - EXO.tau_EXO_left;
    
end

