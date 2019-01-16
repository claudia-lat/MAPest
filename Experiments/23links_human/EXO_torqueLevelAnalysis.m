
%% ====================== EXO_torqueLevelAnalysis =========================
% Extract torques from EXO table and compare with the tauFirst(1,:).
% Note: the exo angles has to be compared with the tauFirst x and no longer
% from tau_MAP.

for blockIdx = 1 : block.nrOfBlocks
    %% -------Right shoulder
    EXO.tmp.tau_EXO_right = zeros(1,size(EXO.CoC(blockIdx).qToCompare_right_round,1));
    for qIdx = 1 : size(EXO.CoC(blockIdx).qToCompare_right_round,1)
        for tableIdx = 1 : size(EXO.extractedDataRaw,1)
            if (EXO.CoC(blockIdx).qToCompare_right_round(qIdx) == EXO.extractedDataRaw(tableIdx,1))
                EXO.tmp.tau_EXO_right(qIdx) = - EXO.extractedDataRaw(tableIdx,EXO.tableInfo(8).range(subjectID));
            end
        end
    end
    
    %% -------Left shoulder
    EXO.tmp.tau_EXO_left = zeros(1,size(EXO.CoC(blockIdx).qToCompare_left_round,1));
    for qIdx = 1 : size(EXO.CoC(blockIdx).qToCompare_left_round,1)
        for tableIdx = 1 : size(EXO.extractedDataRaw,1)
            if (EXO.CoC(blockIdx).qToCompare_left_round(qIdx) == EXO.extractedDataRaw(tableIdx,1))
                EXO.tmp.tau_EXO_left(qIdx) = EXO.extractedDataRaw(tableIdx,EXO.tableInfo(8).range(subjectID));
            end
        end
    end
    
    %% Save into a struct
    exo_tauLevel(blockIdx).block = block.labels(blockIdx);
    exo_tauLevel(blockIdx).masterTime = synchroKin(blockIdx).masterTime;
    exo_tauLevel(blockIdx).torqueFromTable_right = EXO.tmp.tau_EXO_right;
    exo_tauLevel(blockIdx).torqueFromTable_left  = EXO.tmp.tau_EXO_left;
    % tauFirst_MAPest - tau_EXOfromTable
    exo_tauLevel(blockIdx).torqueDiff_right = CoC(blockIdx).Rsho_tauFirst(1,:) - EXO.tmp.tau_EXO_right;
    exo_tauLevel(blockIdx).torqueDiff_left  = CoC(blockIdx).Lsho_tauFirst(1,:) - EXO.tmp.tau_EXO_left;
    
end
