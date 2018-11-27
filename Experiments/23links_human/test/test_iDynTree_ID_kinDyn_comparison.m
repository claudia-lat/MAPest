
% We compute Inverse Dynamics via iDynTree to have a rough comparison
% w.r.t. the MAP computation
iDynIDparams.base = currentBase;

%% =============== Base 6D acceleration expressed w.r.t. G ================
% -------------------------------------------------------------------------

% ======= 3D base ang acc from Xsens suit.links already expressed in G
% 1) extract total ang acc + orientation vectors
for suitLinksIdx = 1 : size(suit.links,1)
    if suit.links{suitLinksIdx, 1}.label == currentBase
        baseAngAcc_tot      = suit.links{suitLinksIdx, 1}.meas.angularAcceleration;
        baseOrientation_tot = suit.links{suitLinksIdx, 1}.meas.orientation;
        break
    end
    break
end

% 2) subdivide vectors into 5 blocks
for blockIdx = 1 : block.nrOfBlocks
    tmp.cutRange{blockIdx} = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
    iDynIDparams.baseAcc(blockIdx).baseAngAcc_wrtG  = baseAngAcc_tot(:,tmp.cutRange{blockIdx});
    iDynIDparams.orientation(blockIdx).baseOrientation = baseOrientation_tot(:,tmp.cutRange{blockIdx});
end
clearvars baseAngAcc_tot baseOrientation_tot;

% ======= 3D base lin acc (free body acc) from Xsens suit.sensors already
% expressed in G, without gravity
for blockIdx = 1 : block.nrOfBlocks
    for suitSensIdx = 1 : size(suit_runtime.sensors,1)
        if suit_runtime.sensors{suitSensIdx, 1}.label == currentBase
            iDynIDparams.baseAcc(blockIdx).baseLinAcc_wrtG = suit_runtime.sensors{suitSensIdx, 1}.meas(blockIdx).sensorFreeAcceleration;
            break
        end
    end
end

% ======= Compose the 6D base acc
for blockIdx = 1 : block.nrOfBlocks
    iDynIDparams.baseAcc(blockIdx).baseAcc_wrtG = zeros(6,size(iDynIDparams.baseAcc(blockIdx).baseAngAcc_wrtG ,2));
    iDynIDparams.baseAcc(blockIdx).baseAcc_wrtG(1:3,:) = iDynIDparams.baseAcc(blockIdx).baseLinAcc_wrtG; %lin
    iDynIDparams.baseAcc(blockIdx).baseAcc_wrtG(4:6,:) = iDynIDparams.baseAcc(blockIdx).baseAngAcc_wrtG; %ang
    %         iDynIDparams.baseAcc(blockIdx).baseAcc(4:6,:) = repmat(0,size(iDynIDparams.baseAcc(blockIdx).baseAcc(1:3,:)));
    %test with ang acc zero
end

%% ===================== 6D fext expressed w.r.t. G =======================
% -------------------------------------------------------------------------

if ~isfield(iDynIDparams,'fext')
    % =============== Left foot
    % 1) Left foot quaternion from suit.links
    for blockIdx = 1 : block.nrOfBlocks
        tmp.cutRange{blockIdx} = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
        for suitLinksIdx = 1 : size(suit.links,1)
            if strcmpi(suit.links{suitLinksIdx, 1}.label,'LeftFoot')
                iDynIDparams.orientation(blockIdx).LeftFootOrientation = ...
                    suit.links{suitLinksIdx, 1}.meas.orientation(:,tmp.cutRange{blockIdx});
                break
            end
        end
    end
    
    % 2) Left foot external force transform
    G_T_leftFootRot = iDynTree.Rotation();
    G_T_leftFootPos = iDynTree.Position();
    G_T_leftFootPos.fromMatlab([0; 0; 0]); %no translation, zero vector const
    for blockIdx = 1 : block.nrOfBlocks
        disp('-------------------------------------------------------------------');
        disp(strcat('Left foot ext forces in G frame for Block ',num2str(blockIdx),'...'));
        len = size(iDynIDparams.orientation(blockIdx).LeftFootOrientation,2);
        for i = 1 : len
            G_R_leftFoot = quat2Mat(iDynIDparams.orientation(blockIdx).LeftFootOrientation(:,i));
            %         figure;
            %         imagesc(G_R_leftFoot);
            G_T_leftFootRot.fromMatlab(G_R_leftFoot);
            G_T_leftFoot = iDynTree.Transform(G_T_leftFootRot, G_T_leftFootPos);
            iDynIDparams.fext(blockIdx).leftShoe_wrtG = ...
                G_T_leftFoot.asAdjointTransformWrench().toMatlab() * shoes(blockIdx).Left_HF;
        end
    end
    
    % =============== Right foot
    % 1) Right foot quaternion from suit.links
    for blockIdx = 1 : block.nrOfBlocks
        tmp.cutRange{blockIdx} = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
        for suitLinksIdx = 1 : size(suit.links,1)
            if strcmpi(suit.links{suitLinksIdx, 1}.label,'RightFoot')
                iDynIDparams.orientation(blockIdx).RightFootOrientation = ...
                    suit.links{suitLinksIdx, 1}.meas.orientation(:,tmp.cutRange{blockIdx});
                break
            end
        end
    end
    
    % 2) Right foot external force transform
    G_T_rightFootRot = iDynTree.Rotation();
    G_T_rightFootPos = iDynTree.Position();
    G_T_rightFootPos.fromMatlab([0; 0; 0]); %no translation, zero vector const
    for blockIdx = 1 : block.nrOfBlocks
        disp('-------------------------------------------------------------------');
        disp(strcat('Right foot ext forces in G frame for Block ',num2str(blockIdx),'...'));
        len = size(iDynIDparams.orientation(blockIdx).LeftFootOrientation,2);
        for i = 1 : len
            G_R_rightFoot = quat2Mat(iDynIDparams.orientation(blockIdx).RightFootOrientation(:,i));
            %         figure;
            %         imagesc(G_R_leftFoot);
            G_T_rightFootRot.fromMatlab(G_R_rightFoot);
            G_T_rightFoot = iDynTree.Transform(G_T_rightFootRot, G_T_rightFootPos);
            iDynIDparams.fext(blockIdx).rightShoe_wrtG = ...
                G_T_rightFoot.asAdjointTransformWrench().toMatlab() * shoes(blockIdx).Right_HF;
        end
    end
end

%% ======================= Base position w.r.t. G =========================
% -------------------------------------------------------------------------

% METODO1 (BAD):    G_pos_base = G_R_L * L_pos_base(from_suit.sensors)
% % Note: Base sensor position used! Not the link one! Tappullo!!
% 
% for blockIdx = 1 : block.nrOfBlocks
%     for suitSensIdx = 1 : size(suit.sensors,1)
%         if suit.sensors{suitSensIdx, 1}.label == currentBase
%             len = size(iDynIDparams.orientation(blockIdx).baseOrientation,2);
%             iDynIDparams.basePosition(blockIdx).basePos_wrtG = zeros(3,len);
%             for i = 1 : len
%                 G_R_L = quat2Mat(iDynIDparams.orientation(blockIdx).baseOrientation(:,i));
%                 iDynIDparams.basePosition(blockIdx).basePos_wrtG(:,i) = G_R_L * suit.sensors{suitSensIdx, 1}.position;
%             end
%             break
%         end
%     end
% end

% METODO 2:    G_pos_base directly from suit.links, already expressed w.r.t. G
% 2.1) Extract total position vector
for suitLinksIdx = 1 : size(suit.links,1)
    if suit.links{suitLinksIdx, 1}.label == currentBase
        basePos_tot  = suit.links{suitLinksIdx, 1}.meas.position;
        break
    end
    break
end

% 2.2) Subdivide vectors into 5 blocks
for blockIdx = 1 : block.nrOfBlocks
    tmp.cutRange{blockIdx} = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
    iDynIDparams.basePosition(blockIdx).basePos_wrtG  = basePos_tot(:,tmp.cutRange{blockIdx});
end
clearvars basePos_tot;

%% ===================== Base 6D velocity w.r.t. G ========================
% -------------------------------------------------------------------------

% ======= 3D base ang vel from Xsens suit.links already expressed w.r.t. G
% 1) extract total ang vel + lin vel
for suitLinksIdx = 1 : size(suit.links,1)
    if suit.links{suitLinksIdx, 1}.label == currentBase
        baseAngVel_tot = suit.links{suitLinksIdx, 1}.meas.angularVelocity;
        baseLinVel_tot = suit.links{suitLinksIdx, 1}.meas.velocity;
        break
    end
    break
end

% 2) subdivide vectors into 5 blocks
for blockIdx = 1 : block.nrOfBlocks
    tmp.cutRange{blockIdx} = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
    iDynIDparams.baseVel(blockIdx).baseAngVel_wrtG  = baseAngVel_tot(:,tmp.cutRange{blockIdx});
    iDynIDparams.baseVel(blockIdx).baseLinVel_wrtG  = baseLinVel_tot(:,tmp.cutRange{blockIdx});
end
clearvars baseAngVel_tot baseLinVel_tot;

% ======= Compose the 6D base vel
for blockIdx = 1 : block.nrOfBlocks
    iDynIDparams.baseVel(blockIdx).baseVel_wrtG = zeros(6,size(iDynIDparams.baseVel(blockIdx).baseAngVel_wrtG ,2));
    iDynIDparams.baseVel(blockIdx).baseVel_wrtG(1:3,:) = iDynIDparams.baseVel(blockIdx).baseLinVel_wrtG; %lin
    iDynIDparams.baseVel(blockIdx).baseVel_wrtG(4:6,:) = iDynIDparams.baseVel(blockIdx).baseAngVel_wrtG; %ang
end

%% ============================ ID computation ============================
% -------------------------------------------------------------------------

for blockIdx = 1 : block.nrOfBlocks
    disp('-------------------------------------------------------------------');
    disp(strcat('[Start] iDynTree ID computation for Block ',num2str(blockIdx),'...'));
    tau_iDynTree(blockIdx).tau = iDynTreeID_kinDyn_floating(human_kinDynComp, ...
        iDynIDparams.orientation(blockIdx).baseOrientation, ...
        iDynIDparams.basePosition(blockIdx).basePos_wrtG, ...
        iDynIDparams.baseVel(blockIdx).baseVel_wrtG, ...
        iDynIDparams.baseAcc(blockIdx).baseAcc_wrtG, ...
        synchroKin(blockIdx), ...
        iDynIDparams.fext(blockIdx));
    disp(strcat('[End] iDynTree ID computation for Block ',num2str(blockIdx)));
end
save(fullfile(bucket.pathToProcessedData,'tau_iDynTree.mat'),'tau_iDynTree');
