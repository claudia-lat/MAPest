
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

%% Computation of torques w.r.t. URDF shoulder (rounded) angles
for blockIdx = 1 : block.nrOfBlocks
    
    exo_tauLevel(blockIdx).block = block.labels(blockIdx);
    exo_tauLevel(blockIdx).masterTime = synchroKin(blockIdx).masterTime;
    exo_tauLevel(blockIdx).torqueFromTable_right = EXO.rightRoundedTable(blockIdx).M_support_mod;
    exo_tauLevel(blockIdx).torqueFromTable_left  = EXO.leftRoundedTable(blockIdx).M_support_mod;
    
    % Final torque-level estimation only for the shoulders:
    %       finalTorque_sho = tauFirst_MAPest_sho - tau_EXOfromTable_sho (new one)
    % with tauFirst_MAPest after the CoC.
    exo_tauLevel(blockIdx).finalTorque_right = CoC(blockIdx).Rsho_tauFirst(1,:) - ...
        EXO.rightRoundedTable(blockIdx).M_support_mod;
    exo_tauLevel(blockIdx).finalTorque_left  = CoC(blockIdx).Lsho_tauFirst(1,:) - ...
        EXO.leftRoundedTable(blockIdx).M_support_mod;
end
