
%% ====================== EXO_torqueLevelAnalysis =========================
% The goal of the torque-level joint torque analysis is to exploit
% the torques already computed in the EXO table.
% The final computation is:
%
%                finalTorque = MAPtorque - EXOtableTorque
%
% where:
% - MAPtorque : is the torque at the shoulder
%               - computed in MAPcomputation_floating.m with the exo weight
%                 at the Pelvis
%               - transformed with a Change of Coordinate (CoC) procedure
%                 in order to match the EXO table values (3D vs. 2D plan).
% -EXOtableTorque : is the torque at the shoulder basically coming from
%                   the EXO table.
%
% The above mention equation has been computed for both the shoulders.
%
% The main limitation of the torque-level approach is that the finalTorque
% can be only computed for the shoulders (only available angles to
% be matched with the CoC).

%% Computation of new torque from table:
%
%   newTableTorque = F_arm_support * 0.6895 * subject upper arm length
%
% Note: The point where the exo applies the force at the upper arm is
%       0.6895 of the total upper arm length (i.e., the application point
%       of the force does not coincide with the elbow joint!!!).

EXO.newTableTorque = zeros(size(EXO.extractedDataRaw,1),1);
for tableIdx = 1 : size(EXO.extractedDataRaw,1)
    EXO.newTableTorque(tableIdx) = EXO.extractedDataRaw(tableIdx,EXO.tableInfo(8).range(subjectID)) * ...
        0.6895 * subjectParamsFromData.leftUpperArm_y;
end

%% Computation of torques w.r.t. URDF shoulder (rounded) angles
for blockIdx = 1 : block.nrOfBlocks
    % right shoulder
    EXO.tmp.tau_EXO_right = zeros(1,size(EXO.CoC(blockIdx).qToCompare_right_round,1));
    for qIdx = 1 : size(EXO.CoC(blockIdx).qToCompare_right_round,1)
        for tableIdx = 1 : size(EXO.extractedDataRaw,1)
            if (EXO.CoC(blockIdx).qToCompare_right_round(qIdx) == EXO.extractedDataRaw(tableIdx,1))
                EXO.tmp.tau_EXO_right(qIdx) = - EXO.newTableTorque(tableIdx,1);
            end
        end
    end
    
    % left shoulder
    EXO.tmp.tau_EXO_left = zeros(1,size(EXO.CoC(blockIdx).qToCompare_left_round,1));
    for qIdx = 1 : size(EXO.CoC(blockIdx).qToCompare_left_round,1)
        for tableIdx = 1 : size(EXO.extractedDataRaw,1)
            if (EXO.CoC(blockIdx).qToCompare_left_round(qIdx) == EXO.extractedDataRaw(tableIdx,1))
                EXO.tmp.tau_EXO_left(qIdx) = EXO.newTableTorque(tableIdx,1); % sign changed because of mirrored torque
            end
        end
    end
    
    %% Save into a struct
    exo_tauLevel(blockIdx).block = block.labels(blockIdx);
    exo_tauLevel(blockIdx).masterTime = synchroKin(blockIdx).masterTime;
    exo_tauLevel(blockIdx).torqueFromTable_right = EXO.tmp.tau_EXO_right;
    exo_tauLevel(blockIdx).torqueFromTable_left  = EXO.tmp.tau_EXO_left;

    % Final torque-level estimation only for the shoulders:
    %
    %    finalTorque_sho = tauFirst_MAPest_sho - tau_EXOfromTable_sho (new one)
    %
    % with tauFirst_MAPest after the CoC
    exo_tauLevel(blockIdx).finalTorque_right = CoC(blockIdx).Rsho_tauFirst(1,:) - EXO.tmp.tau_EXO_right;
    exo_tauLevel(blockIdx).finalTorque_left  = CoC(blockIdx).Lsho_tauFirst(1,:) - EXO.tmp.tau_EXO_left;
    
end
