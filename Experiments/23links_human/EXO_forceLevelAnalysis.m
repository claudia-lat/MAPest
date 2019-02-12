
%% ======================= EXO_forceLevelAnalysis =========================
% The goal of the force-level joint torque analysis is to consider how the
% EXO forces affect the whole-body dynamics.
% The final computation is:
%
%          exo_forceLevel = MAPtorque - pinv(NB)N (J' * EXOforces)
%
% where:
% - MAPtorque : is the whole-body torque
%               - computed in MAPcomputation_floating.m without the exo
%                 weight in the URDF
% - pinv(NB)N : is the term for the feet implicit constraints which makes
%               the dynamics of the system to satisfy the 2-feet contact
%               constraint
% - (J' * EXOforces) is:   (J_LUA'*G_f_LUA + J_LH'*G_f_LH +
%                          J_RUA'*G_f_RUA + J_RH'*G_f_RH)
%    where
%    - each force is the force of the exo (cin the table) transformed in
%      the global frame G,
%    - each J' is the jacobian when considering new frames for the exo
%      contact points.

% Legend:
% ---------------------
% LUA = Left Upper Arm
% LH  = Left Hip
% RUA = Right Upper Arm
% RH  = Right Hip
% ---------------------


%% Compute the Jacobians for the EXO contact points (i.e., J_exo)

% ----Definition of new frames for the exo at the human model level (not in the URDF)
% -------LUA
EXO.tmp.exoLUAdistance = subjectParamsFromData.leftUpperArm_y * 0.6895;
% pos
EXO.tmp.LUA_T_exoLUAPos = iDynTree.Position();
EXO.tmp.LUA_T_exoLUAPos.fromMatlab([0; EXO.tmp.exoLUAdistance; 0]); %exoLUAseenFromLUA
% rot
EXO.tmp.LUA_T_exoLUARot = iDynTree.Rotation;
EXO.tmp.LUA_T_exoLUARot.fromMatlab([ 1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0; ...
    0.0,  0.0,  1.0]);
% transf
EXO.tmp.LUA_T_exoLUA = iDynTree.Transform(EXO.tmp.LUA_T_exoLUARot,EXO.tmp.LUA_T_exoLUAPos);
humanModel.addAdditionalFrameToLink('LeftUpperArm','LeftUpperArm_exo', EXO.tmp.LUA_T_exoLUA);

% -------LH
EXO.tmp.exoLHdistance = subjectParamsFromData.pelvisBox(2)/2;
% pos
EXO.tmp.Hip_T_exoLHPos = iDynTree.Position();
EXO.tmp.Hip_T_exoLHPos.fromMatlab([0; EXO.tmp.exoLHdistance; 0]); %exoRHseenFromHip
% rot
EXO.tmp.Hip_T_exoLHRot = iDynTree.Rotation;
EXO.tmp.Hip_T_exoLHRot.fromMatlab([ 1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0; ...
    0.0,  0.0,  1.0]);
% transf
EXO.tmp.Hip_T_exoLH = iDynTree.Transform(EXO.tmp.Hip_T_exoLHRot,EXO.tmp.Hip_T_exoLHPos);
humanModel.addAdditionalFrameToLink('Pelvis','LeftHip_exo', EXO.tmp.Hip_T_exoLH);

% -------RUA
EXO.tmp.exoRUAdistance = EXO.tmp.exoLUAdistance; %because of the symmetry
% pos
EXO.tmp.RUA_T_exoRUAPos = iDynTree.Position();
EXO.tmp.RUA_T_exoRUAPos.fromMatlab([0; -EXO.tmp.exoRUAdistance; 0]); %exoRUAseenFromRUA
% rot
EXO.tmp.RUA_T_exoRUARot = iDynTree.Rotation;
EXO.tmp.RUA_T_exoRUARot.fromMatlab([ 1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0; ...
    0.0,  0.0,  1.0]);
% transf
EXO.tmp.RUA_T_exoRUA = iDynTree.Transform(EXO.tmp.RUA_T_exoRUARot,EXO.tmp.RUA_T_exoRUAPos);
humanModel.addAdditionalFrameToLink('RightUpperArm','RightUpperArm_exo', EXO.tmp.RUA_T_exoRUA);

% -------RH
EXO.tmp.exoRHdistance = EXO.tmp.exoLHdistance; %because of the symmetry
% pos
% pos
EXO.tmp.Hip_T_exoRHPos = iDynTree.Position();
EXO.tmp.Hip_T_exoRHPos.fromMatlab([0; -EXO.tmp.exoRHdistance; 0]); %exoRHseenFromHip
% rot
EXO.tmp.Hip_T_exoRHRot = iDynTree.Rotation;
EXO.tmp.Hip_T_exoRHRot.fromMatlab([ 1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0; ...
    0.0,  0.0,  1.0]);
% transf
EXO.tmp.Hip_T_exoRH = iDynTree.Transform(EXO.tmp.Hip_T_exoRHRot,EXO.tmp.Hip_T_exoRHPos);
humanModel.addAdditionalFrameToLink('Pelvis','RightHip_exo', EXO.tmp.Hip_T_exoRH);

EXO.tmp.frameWhereFexoIsActing = {'LeftUpperArm_exo';
    'LeftHip_exo';
    'RightUpperArm_exo';
    'RightHip_exo'};

% Tailored kinDyn object
exo_kinDynComp = iDynTree.KinDynComputations();
exo_kinDynComp.loadRobotModel(humanModel);
exo_kinDynComp.setFloatingBase(currentBase);

if ~exist(fullfile(bucket.pathToProcessedData,'J_exo.mat'), 'file')
    % ----Computation of full Jacobians
    for frameExoIdx = 1 : length(EXO.tmp.frameWhereFexoIsActing)
        for blockIdx = 1 : block.nrOfBlocks
            disp('---------------------------');
            disp(strcat(EXO.tmp.frameWhereFexoIsActing{frameExoIdx},' Jacobians computation for Block ',num2str(blockIdx)));
            J_exo(blockIdx).block = block.labels(blockIdx);
            len = length(synchroKin(blockIdx).masterTime);
            
            if frameExoIdx == 1
                J_exo(blockIdx).LUA = cell(len,1);
            end
            if frameExoIdx == 2
                J_exo(blockIdx).LH  = cell(len,1);
            end
            if frameExoIdx == 3
                J_exo(blockIdx).RUA = cell(len,1);
            end
            if frameExoIdx == 4
                J_exo(blockIdx).RH  = cell(len,1);
            end
            
            EXO.tmp.frameFexo = EXO.tmp.frameWhereFexoIsActing{frameExoIdx};
            iDynTreeJacobian = iDynTree.FrameFreeFloatingJacobian(exo_kinDynComp.model);
            iDynTreeJacobian.zero();
            
            q  = iDynTree.JointPosDoubleArray(exo_kinDynComp.model);
            dq = iDynTree.JointDOFsDoubleArray(exo_kinDynComp.model);
            baseVelocity = iDynTree.Twist();
            
            gravity = iDynTree.Vector3();
            gravity.fromMatlab([0; 0; -9.81]);
            %             gravityZero = iDynTree.Vector3();
            %             gravityZero.zero();
            
            baseKinDynModel = exo_kinDynComp.getFloatingBase();
            % Consistency check
            % berdy.model base and exo_kinDynComp.model have to be consistent!
            if currentBase ~= baseKinDynModel
                error(strcat('[ERROR] The berdy model base (',currentBase,') and the kinDyn model base (',baseKinDynModel,') do not match!'));
            end
            
            for i = 1 : len
                q.fromMatlab(synchroKin(blockIdx).q(:,i));
                dq.fromMatlab(synchroKin(blockIdx).dq(:,i));
                baseVelocity.fromMatlab([baseVel(blockIdx).baseLinVelocity(:,i); ...
                    baseVel(blockIdx).baseAngVelocity(:,i)]);
                
                % Compute the Jacobian J = [J(q)_b J(q)_s] from kinDyn
                exo_kinDynComp.setRobotState(G_T_base(blockIdx).G_T_b{i,1},q,baseVelocity,dq,gravity);
                exo_kinDynComp.getFrameFreeFloatingJacobian(EXO.tmp.frameFexo, iDynTreeJacobian);
                
                EXO.tmp.fullJacobian = iDynTreeJacobian.toMatlab();
                if frameExoIdx == 1
                    J_exo(blockIdx).LUA{i,1} = EXO.tmp.fullJacobian(:,:);
                end
                if frameExoIdx == 2
                    J_exo(blockIdx).LH{i,1}  = EXO.tmp.fullJacobian(:,:);
                end
                if frameExoIdx == 3
                    J_exo(blockIdx).RUA{i,1} = EXO.tmp.fullJacobian(:,:);
                end
                if frameExoIdx == 4
                    J_exo(blockIdx).RH{i,1}  = EXO.tmp.fullJacobian(:,:);
                end
            end
        end
    end
    save(fullfile(bucket.pathToProcessedData,'J_exo.mat'),'J_exo');
else
    load(fullfile(bucket.pathToProcessedData,'J_exo.mat'));
end

%% Compute rotation matrices

% 1) Const rotation LUAexo_R_LUAtable
% Left arm : - LeftUpperArm_exo (i.e., LUAexo)     frame for the exo on the URDF
%            - LeftUpperArm_table (i.e., LUAtable) frame for the table data (expressed w.r.t. arm frame)
EXO.LUAexo_R_LUAtable = [ 0.0,  0.0,  1.0; ...
    1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0];

% 2) Const rotation LHexo_R_LHtable
% Left hip : - LefHip_exo (i.e., LUAexo)      frame for the exo on the URDF
%            - LeftHip_table (i.e., LUAtable) frame for the table data (expressed w.r.t. torso frame)
EXO.LHexo_R_LHtable = [ 0.0,  0.0,  -1.0; ...
    -1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0];

% 3) Const rotation RUAexo_R_RUAtable
% Right arm : - RightUpperArm_exo (i.e., RUAexo)     frame for the exo on the URDF
%             - RightUpperArm_table (i.e., RUAtable) frame for the table data (expressed w.r.t. arm frame)
EXO.RUAexo_R_RUAtable = [ 0.0,  0.0,  -1.0; ...
    -1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0];

% 4) Const rotation RHexo_R_RHtable
% Right hip : - RightHip_exo (i.e., LUAexo)     frame for the exo on the URDF
%             - RightHip_table (i.e., LUAtable) frame for the table data (expressed w.r.t. torso frame)
EXO.RHexo_R_RHtable = [ 0.0,  0.0,  1.0; ...
    1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0];

% 5) Varying rotations expressed w.r.t. the world frame G (as required in the dynamic equation)
if ~exist(fullfile(bucket.pathToProcessedData,'G_R_exoFrames.mat'), 'file')
    for blockIdx = 1 : block.nrOfBlocks
        len = length(synchroKin(blockIdx).masterTime);
        G_R_exoFrames(blockIdx).block = block.labels(blockIdx);
        
        q  = iDynTree.JointPosDoubleArray(exo_kinDynComp.model);
        dq = iDynTree.JointDOFsDoubleArray(exo_kinDynComp.model);
        baseVelocity = iDynTree.Twist();
        
        gravity = iDynTree.Vector3();
        gravity.fromMatlab([0; 0; -9.81]);
        %     gravityZero = iDynTree.Vector3();
        %     gravityZero.zero();
        
        for i = 1 : len
            q.fromMatlab(synchroData(blockIdx).q(:,i));
            dq.fromMatlab(synchroData(blockIdx).dq(:,i));
            baseVelocity.fromMatlab([baseVel(blockIdx).baseLinVelocity(:,i); ...
                baseVel(blockIdx).baseAngVelocity(:,i)]);
            
            exo_kinDynComp.setRobotState(G_T_base(blockIdx).G_T_b{i,1},q,baseVelocity,dq,gravity);
            % 5.1) G_R_LUAexo
            EXO.tmp.G_T_LUAexo = exo_kinDynComp.getWorldTransform('LeftUpperArm_exo');
            EXO.tmp.G_H_LUAexo = EXO.tmp.G_T_LUAexo.asHomogeneousTransform().toMatlab();
            G_R_exoFrames(blockIdx).G_R_LUAexo{i,1} = EXO.tmp.G_H_LUAexo(1:3,1:3);
            % 5.2) G_R_RUAexo
            EXO.tmp.G_T_RUAexo = exo_kinDynComp.getWorldTransform('RightUpperArm_exo');
            EXO.tmp.G_H_RUAexo = EXO.tmp.G_T_RUAexo.asHomogeneousTransform().toMatlab();
            G_R_exoFrames(blockIdx).G_R_RUAexo{i,1} = EXO.tmp.G_H_RUAexo(1:3,1:3);
            % 5.3) G_R_LHexo
            EXO.tmp.G_T_LHexo = exo_kinDynComp.getWorldTransform('LeftHip_exo');
            EXO.tmp.G_H_LHexo = EXO.tmp.G_T_LHexo.asHomogeneousTransform().toMatlab();
            G_R_exoFrames(blockIdx).G_R_LHexo{i,1} = EXO.tmp.G_H_LHexo(1:3,1:3);
            % 5.4) G_R_RHexo
            EXO.tmp.G_T_RHexo = exo_kinDynComp.getWorldTransform('RightHip_exo');
            EXO.tmp.G_H_RHexo = EXO.tmp.G_T_RHexo.asHomogeneousTransform().toMatlab();
            G_R_exoFrames(blockIdx).G_R_RHexo{i,1} = EXO.tmp.G_H_RHexo(1:3,1:3);
        end
    end
    save(fullfile(bucket.pathToProcessedData,'G_R_exoFrames.mat'),'G_R_exoFrames');
else
    load(fullfile(bucket.pathToProcessedData,'G_R_exoFrames.mat'));
end

%% Force transformation
% ----Match forces from EXO extractedTable with the angles.
for blockIdx = 1 : block.nrOfBlocks
    len = size(synchroKin(blockIdx).masterTime ,2);
    EXO.exoForcesURDFcompatible(blockIdx).block = block.labels(blockIdx);
    
    EXO.tmp.f_table_LUA = zeros(3,len);
    EXO.tmp.f_table_LH  = zeros(3,len);
    for qIdx = 1 : len
        for tableIdx = 1 : size(EXO.extractedTable(1).shoulder_angles,1)
            if (EXO.CoC(blockIdx).qToCompare_left_round(qIdx) == EXO.extractedTable(subjectID).shoulder_angles(tableIdx,1))
                % -------Left upper arm
                EXO.tmp.f_table_LUA(1,qIdx) = EXO.extractedTable(subjectID).F_arm_scher(tableIdx,1);
                EXO.tmp.f_table_LUA(2,qIdx) = EXO.extractedTable(subjectID).F_arm_support(tableIdx,1);
                % -------Left hip
                EXO.tmp.f_table_LH(1,qIdx) = EXO.extractedTable(subjectID).F_KGkraft_x(tableIdx,1);
                EXO.tmp.f_table_LH(2,qIdx) = EXO.extractedTable(subjectID).F_KGkraft_y(tableIdx,1);
            end
        end
    end
    % Transformations
    for sampleIdx = 1:len
        EXO.exoForcesURDFcompatible(blockIdx).LUA(:,sampleIdx) = G_R_exoFrames(blockIdx).G_R_LUAexo{sampleIdx,1} * ...
            EXO.LUAexo_R_LUAtable * EXO.tmp.f_table_LUA(:,sampleIdx);
        EXO.exoForcesURDFcompatible(blockIdx).LH(:,sampleIdx) = G_R_exoFrames(blockIdx).G_R_LHexo{sampleIdx,1} * ...
            EXO.LHexo_R_LHtable * EXO.tmp.f_table_LH(:,sampleIdx);
    end
    
    EXO.tmp.f_table_RUA = zeros(3,len);
    EXO.tmp.f_table_RH  = zeros(3,len);
    for qIdx = 1 : len
        for tableIdx = 1 : size(EXO.extractedTable(1).shoulder_angles,1)
            if (EXO.CoC(blockIdx).qToCompare_right_round(qIdx) == EXO.extractedTable(subjectID).shoulder_angles(tableIdx,1))
                % -------Right upper arm
                EXO.tmp.f_table_RUA(1,qIdx) = EXO.extractedTable(subjectID).F_arm_scher(tableIdx,1); % sign? to be investigated
                EXO.tmp.f_table_RUA(2,qIdx) = EXO.extractedTable(subjectID).F_arm_support(tableIdx,1); % sign? to be investigated
                % -------Right hip
                EXO.tmp.f_table_RH(1,qIdx) = EXO.extractedTable(subjectID).F_KGkraft_x(tableIdx,1); % sign? to be investigated
                EXO.tmp.f_table_RH(2,qIdx) = EXO.extractedTable(subjectID).F_KGkraft_y(tableIdx,1); % sign? to be investigated
            end
        end
    end
    % Transformations
    for sampleIdx = 1:len
        EXO.exoForcesURDFcompatible(blockIdx).RUA(:,sampleIdx) = G_R_exoFrames(blockIdx).G_R_RUAexo{sampleIdx,1} * ...
            EXO.RUAexo_R_RUAtable * EXO.tmp.f_table_RUA(:,sampleIdx);
        EXO.exoForcesURDFcompatible(blockIdx).RH(:,sampleIdx) = G_R_exoFrames(blockIdx).G_R_RHexo{sampleIdx,1} * ...
            EXO.RHexo_R_RHtable * EXO.tmp.f_table_RH(:,sampleIdx);
    end
end

%% Computation of the final torque
% Computation of finalTorque = MAPtorque - pinv(NB)N (J' * EXOforces)
% More in detail, the term (J' * EXOforces) is:
% (J_LUA'*G_f_LUA + J_LH'*G_f_LH + J_RUA'*G_f_RUA + J_RH'*G_f_RH)

% Load the term pinv(NB)N already computed
load(fullfile(bucket.pathToProcessedData,'implicitFeetContraint.mat'));

for blockIdx = 1 : block.nrOfBlocks
    len = size(synchroKin(blockIdx).masterTime ,2);
    
    exo_forceLevel(blockIdx).block = block.labels(blockIdx);
    for sampleIdx = 1 : len
        exo_forceLevel(blockIdx).torqueEXO(:,sampleIdx) = implFeetConstraint(blockIdx).term{sampleIdx, 1} * ... % implicit term pinv(NB)N
            (J_exo(blockIdx).LUA{sampleIdx,1}(1:3,:)' * EXO.exoForcesURDFcompatible(blockIdx).LUA(:,sampleIdx) + ... % LUA
            J_exo(blockIdx).LH{sampleIdx,1}(1:3,:)' * EXO.exoForcesURDFcompatible(blockIdx).LH(:,sampleIdx) + ... % LH
            J_exo(blockIdx).RUA{sampleIdx,1}(1:3,:)'  * EXO.exoForcesURDFcompatible(blockIdx).RUA(:,sampleIdx) + ... % RUA
            J_exo(blockIdx).RH{sampleIdx,1}(1:3,:)' * EXO.exoForcesURDFcompatible(blockIdx).RH(:,sampleIdx)); % RH
    end
    
    for sampleIdx = 1 : len
        exo_forceLevel(blockIdx).torque(:,sampleIdx) = estimatedVariables.tau(blockIdx).values(:,sampleIdx) - ... % MAP estimation
            exo_forceLevel(blockIdx).torqueEXO(:,sampleIdx); % compuetd EXO torque
    end
end

%% Clean up workspace
EXO = rmfield(EXO,'tmp');
