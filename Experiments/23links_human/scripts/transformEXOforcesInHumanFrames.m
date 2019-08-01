
% Legend:
% ---------------------
% LUA = Left Upper Arm
% LH  = Left Hip
% RUA = Right Upper Arm
% RH  = Right Hip
% ---------------------

%% 6D forces from the exo table
for blockIdx = 1 : block.nrOfBlocks
    len = length(synchroKin(blockIdx).masterTime);
    
    EXO.exoForces(blockIdx).block  = block.labels(blockIdx);
    % LUA
    EXO.exoForces(blockIdx).f_table_LUA = zeros(6,len);
    EXO.exoForces(blockIdx).f_table_LUA(1,:) = EXO.leftRoundedTable(blockIdx).F_arm_scher;
    EXO.exoForces(blockIdx).f_table_LUA(2,:) = EXO.leftRoundedTable(blockIdx).F_arm_support;
    EXO.exoForces(blockIdx).f_table_LUA(6,:) = EXO.leftRoundedTable(blockIdx).M_support_mod;
    
    % LH
    EXO.exoForces(blockIdx).f_table_LH  = zeros(6,len);
    EXO.exoForces(blockIdx).f_table_LH(1,:) = EXO.leftRoundedTable(blockIdx).F_KGkraft_x;
    EXO.exoForces(blockIdx).f_table_LH(2,:) = EXO.leftRoundedTable(blockIdx).F_KGkraft_y;
    % EXO.exoForces(blockIdx).f_table_LH(6,:) == 0!! Wrong assumption!
    
    % RUA
    EXO.exoForces(blockIdx).f_table_RUA = zeros(6,len);
    EXO.exoForces(blockIdx).f_table_RUA(1,:) = EXO.rightRoundedTable(blockIdx).F_arm_scher;
    EXO.exoForces(blockIdx).f_table_RUA(2,:) = EXO.rightRoundedTable(blockIdx).F_arm_support;
    EXO.exoForces(blockIdx).f_table_RUA(6,:) = EXO.rightRoundedTable(blockIdx).M_support_mod;
    
    % RH
    EXO.exoForces(blockIdx).f_table_RH  = zeros(6,len);
    EXO.exoForces(blockIdx).f_table_RH(1,:) = EXO.leftRoundedTable(blockIdx).F_KGkraft_x;
    EXO.exoForces(blockIdx).f_table_RH(2,:) = EXO.leftRoundedTable(blockIdx).F_KGkraft_y;
    % EXO.exoForces(blockIdx).f_table_RH(6,:) == 0!! Wrong assumption!
end

%% Transform exo table 6D forces into human frames
% Transform f_table from table frames to URDF frames

% ================================= ARMS ==================================
% ------------------------- from LUAtable to LUA --------------------------
EXO.LUA_R_LUAtable = [ 0.0,  0.0,  1.0; ...
    1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0];
EXO.tmp.exoLUAdistance = subjectParamsFromData.leftUpperArm_y * 0.6895;
EXO.tmp.LUAexoSeenFromLUA = [0; EXO.tmp.exoLUAdistance; 0]; %exoLUAseenFromLUA

EXO.LUA_T_LUAtableRot = iDynTree.Rotation();
EXO.LUA_T_LUAtableRot.fromMatlab(EXO.LUA_R_LUAtable);
EXO.LUA_T_LUAtablePos = iDynTree.Position();
EXO.LUA_T_LUAtablePos.fromMatlab(EXO.tmp.LUAexoSeenFromLUA);
EXO.LUA_T_LUAtable = iDynTree.Transform(EXO.LUA_T_LUAtableRot, EXO.LUA_T_LUAtablePos);

% -------------------------- from RUAtable to RUA -------------------------
EXO.RUA_R_RUAtable = [ 0.0,  0.0,  -1.0; ...
    -1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0];
EXO.tmp.exoRUAdistance = EXO.tmp.exoLUAdistance; %because of the symmetry
EXO.tmp.RUAexoSeenFromRUA = [0; -EXO.tmp.exoLUAdistance; 0]; %exoRUAseenFromRUA

EXO.RUA_T_RUAtableRot = iDynTree.Rotation();
EXO.RUA_T_RUAtableRot.fromMatlab(EXO.RUA_R_RUAtable);
EXO.RUA_T_RUAtablePos = iDynTree.Position();
EXO.RUA_T_RUAtablePos.fromMatlab(EXO.tmp.RUAexoSeenFromRUA);
EXO.RUA_T_RUAtable = iDynTree.Transform(EXO.RUA_T_RUAtableRot, EXO.RUA_T_RUAtablePos);

% ================================= HIPS ==================================
% -------------------------- from LHtable to LH ---------------------------
EXO.LH_R_LHtable = [ 0.0,  0.0,  -1.0; ...
    -1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0];
EXO.tmp.exoLHdistance = subjectParamsFromData.pelvisBox(2)/2; %half of the y length of the pelvis box
EXO.tmp.LHexoSeenFromLH = [0; EXO.tmp.exoLHdistance; 0]; %exoLHseenFromLH

EXO.LH_T_LHtableRot = iDynTree.Rotation();
EXO.LH_T_LHtableRot.fromMatlab(EXO.LH_R_LHtable);
EXO.LH_T_LHtablePos = iDynTree.Position();
EXO.LH_T_LHtablePos.fromMatlab(EXO.tmp.LHexoSeenFromLH);
EXO.LH_T_LHtable = iDynTree.Transform(EXO.LH_T_LHtableRot, EXO.LH_T_LHtablePos);

% -------------------------- from RHtable to RH ---------------------------
EXO.RH_R_RHtable = [ 0.0,  0.0,  1.0; ...
    1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0];
EXO.tmp.exoRHdistance = subjectParamsFromData.pelvisBox(2)/2; %because of the symmetry
EXO.tmp.RHexoSeenFromRH = [0; -EXO.tmp.exoRHdistance; 0]; %exoRHseenFromRH

EXO.RH_T_RHtableRot = iDynTree.Rotation();
EXO.RH_T_RHtableRot.fromMatlab(EXO.RH_R_RHtable);
EXO.RH_T_RHtablePos = iDynTree.Position();
EXO.RH_T_RHtablePos.fromMatlab(EXO.tmp.RHexoSeenFromRH);
EXO.RH_T_RHtable = iDynTree.Transform(EXO.RH_T_RHtableRot, EXO.RH_T_RHtablePos);

% Transformation of Arm and Hips forces
for blockIdx = 1 : block.nrOfBlocks
    len = length(synchroKin(blockIdx).masterTime);
    for i = 1 : len
        EXO.transformedForces(blockIdx).block  = block.labels(blockIdx);
        % arms
        EXO.transformedForces(blockIdx).f_LUA(:,i) = (EXO.LUA_T_LUAtable.asAdjointTransformWrench().toMatlab() * ...
            EXO.exoForces(blockIdx).f_table_LUA(:,i));
        EXO.transformedForces(blockIdx).f_RUA(:,i) = (EXO.RUA_T_RUAtable.asAdjointTransformWrench().toMatlab() * ...
            EXO.exoForces(blockIdx).f_table_RUA(:,i));
        % hips
        EXO.transformedForces(blockIdx).f_LH(:,i) = (EXO.LH_T_LHtable.asAdjointTransformWrench().toMatlab() * ...
            EXO.exoForces(blockIdx).f_table_LH(:,i));
        EXO.transformedForces(blockIdx).f_RH(:,i) = (EXO.RH_T_RHtable.asAdjointTransformWrench().toMatlab() * ...
            EXO.exoForces(blockIdx).f_table_RH(:,i));
    end
end

%% Final format for packaging
% - 1x force on the LeftUpperArm
% - 1x force on the RightUpperArm
% - 1x force at the Pelvis (sum of the two exo forces at the hips)
for blockIdx = 1 : block.nrOfBlocks
    EXO.fext(blockIdx).block  = block.labels(blockIdx);
    % arms
    EXOfext(blockIdx).LUA = EXO.transformedForces(blockIdx).f_LUA;
    EXOfext(blockIdx).RUA = EXO.transformedForces(blockIdx).f_RUA;
    % hips
    EXOfext(blockIdx).PELVIS = EXO.transformedForces(blockIdx).f_LH + EXO.transformedForces(blockIdx).f_RH;
end
save(fullfile(bucket.pathToProcessedData,'EXOfext.mat'),'EXOfext');
