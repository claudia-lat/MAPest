
% Script to collect params usefull for the Inverse Dynamics comparison
% testing

IDcomparisonParams.base = currentBase;

% Extract suit_runtime of the currentBase
for blockIdx = 1 : block.nrOfBlocks
    for suitSensIdx = 1 : size(suit_runtime.sensors,1)
        if suit_runtime.sensors{suitSensIdx, 1}.label == currentBase
            IDcomparisonParams.sensorsFromBase.sensors{suitSensIdx, 1}.meas(blockIdx).sensorOrientation = ...
                suit_runtime.sensors{suitSensIdx, 1}.meas(blockIdx).sensorOrientation;
            IDcomparisonParams.sensorsFromBase.sensors{suitSensIdx, 1}.meas(blockIdx).sensorFreeAcceleration = ...
                suit_runtime.sensors{suitSensIdx, 1}.meas(blockIdx).sensorFreeAcceleration;
            IDcomparisonParams.sensorsFromBase.sensors{suitSensIdx, 1}.meas(blockIdx).sensorOldAcceleration = ...
                suit_runtime.sensors{suitSensIdx, 1}.meas(blockIdx).sensorOldAcceleration;
            break
        end
    end
end

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
    IDcomparisonParams.baseAcc(blockIdx).baseAngAcc_wrtG  = baseAngAcc_tot(:,tmp.cutRange{blockIdx});
    IDcomparisonParams.orientation(blockIdx).baseOrientation = baseOrientation_tot(:,tmp.cutRange{blockIdx});
end
clearvars baseAngAcc_tot baseOrientation_tot;

% ======= 3D base lin acc (free body acc) from Xsens suit.sensors already
% expressed in G, without gravity
for blockIdx = 1 : block.nrOfBlocks
    IDcomparisonParams.baseAcc(blockIdx).baseLinAcc_wrtG = ...
        IDcomparisonParams.sensorsFromBase.sensors{1, 1}.meas(blockIdx).sensorFreeAcceleration;
end

% ======= Compose the 6D base acc
for blockIdx = 1 : block.nrOfBlocks
    IDcomparisonParams.baseAcc(blockIdx).baseAcc_wrtG = zeros(6,size(IDcomparisonParams.baseAcc(blockIdx).baseAngAcc_wrtG ,2));
    IDcomparisonParams.baseAcc(blockIdx).baseAcc_wrtG(1:3,:) = IDcomparisonParams.baseAcc(blockIdx).baseLinAcc_wrtG; %lin
    IDcomparisonParams.baseAcc(blockIdx).baseAcc_wrtG(4:6,:) = IDcomparisonParams.baseAcc(blockIdx).baseAngAcc_wrtG; %ang
    %         IDcomparisonParams.baseAcc(blockIdx).baseAcc(4:6,:) = repmat(0,size(IDcomparisonParams.baseAcc(blockIdx).baseAcc(1:3,:)));
    %test with ang acc zero
end

%% ===================== 6D fext expressed w.r.t. G =======================
% -------------------------------------------------------------------------

if ~isfield(IDcomparisonParams,'fext')
    % =============== Left foot
    % 1) Left foot quaternion from suit.links
    for blockIdx = 1 : block.nrOfBlocks
        tmp.cutRange{blockIdx} = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
        for suitLinksIdx = 1 : size(suit.links,1)
            if strcmpi(suit.links{suitLinksIdx, 1}.label,'LeftFoot')
                IDcomparisonParams.orientation(blockIdx).LeftFootOrientation = ...
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
        len = size(IDcomparisonParams.orientation(blockIdx).LeftFootOrientation,2);
        for i = 1 : len
            G_R_leftFoot = quat2Mat(IDcomparisonParams.orientation(blockIdx).LeftFootOrientation(:,i));
            %         figure;
            %         imagesc(G_R_leftFoot);
            G_T_leftFootRot.fromMatlab(G_R_leftFoot);
            G_T_leftFoot = iDynTree.Transform(G_T_leftFootRot, G_T_leftFootPos);
            IDcomparisonParams.fext(blockIdx).leftShoe_wrtG = ...
                G_T_leftFoot.asAdjointTransformWrench().toMatlab() * shoes(blockIdx).Left_HF;
        end
    end
    
    % =============== Right foot
    % 1) Right foot quaternion from suit.links
    for blockIdx = 1 : block.nrOfBlocks
        tmp.cutRange{blockIdx} = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
        for suitLinksIdx = 1 : size(suit.links,1)
            if strcmpi(suit.links{suitLinksIdx, 1}.label,'RightFoot')
                IDcomparisonParams.orientation(blockIdx).RightFootOrientation = ...
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
        len = size(IDcomparisonParams.orientation(blockIdx).LeftFootOrientation,2);
        for i = 1 : len
            G_R_rightFoot = quat2Mat(IDcomparisonParams.orientation(blockIdx).RightFootOrientation(:,i));
            %         figure;
            %         imagesc(G_R_leftFoot);
            G_T_rightFootRot.fromMatlab(G_R_rightFoot);
            G_T_rightFoot = iDynTree.Transform(G_T_rightFootRot, G_T_rightFootPos);
            IDcomparisonParams.fext(blockIdx).rightShoe_wrtG = ...
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
%             len = size(IDcomparisonParams.orientation(blockIdx).baseOrientation,2);
%             IDcomparisonParams.basePosition(blockIdx).basePos_wrtG = zeros(3,len);
%             for i = 1 : len
%                 G_R_L = quat2Mat(IDcomparisonParams.orientation(blockIdx).baseOrientation(:,i));
%                 IDcomparisonParams.basePosition(blockIdx).basePos_wrtG(:,i) = G_R_L * suit.sensors{suitSensIdx, 1}.position;
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
    IDcomparisonParams.basePosition(blockIdx).basePos_wrtG  = basePos_tot(:,tmp.cutRange{blockIdx});
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
    IDcomparisonParams.baseVel(blockIdx).baseAngVel_wrtG  = baseAngVel_tot(:,tmp.cutRange{blockIdx});
    IDcomparisonParams.baseVel(blockIdx).baseLinVel_wrtG  = baseLinVel_tot(:,tmp.cutRange{blockIdx});
end
clearvars baseAngVel_tot baseLinVel_tot;

% ======= Compose the 6D base vel
for blockIdx = 1 : block.nrOfBlocks
    IDcomparisonParams.baseVel(blockIdx).baseVel_wrtG = zeros(6,size(IDcomparisonParams.baseVel(blockIdx).baseAngVel_wrtG ,2));
    IDcomparisonParams.baseVel(blockIdx).baseVel_wrtG(1:3,:) = IDcomparisonParams.baseVel(blockIdx).baseLinVel_wrtG; %lin
    IDcomparisonParams.baseVel(blockIdx).baseVel_wrtG(4:6,:) = IDcomparisonParams.baseVel(blockIdx).baseAngVel_wrtG; %ang
end


%% ========================= Params w.r.t. Base ===========================
% -------------------------------------------------------------------------

G_gravity = [0; 0; -9.81];
for blockIdx = 1 : block.nrOfBlocks
    len = size(IDcomparisonParams.orientation(1).baseOrientation,2);
    for i = 1 : len
        G_R_base       = quat2Mat(IDcomparisonParams.orientation(blockIdx).baseOrientation(:,i));
        G_R_baseSensor = quat2Mat(IDcomparisonParams.sensorsFromBase.sensors{1, 1}.meas(blockIdx).sensorOrientation(:,i));
        
        % -----------Base 3D Proper Acc: base_properAcc = base_R_G * (G_a_base - G_g)
        % where G_a_base == senor free body acc
          IDcomparisonParams.baseAcc(blockIdx).baseProperAcc_wrtBase = G_R_base' * ...
              (IDcomparisonParams.sensorsFromBase.sensors{1, 1}.meas(blockIdx).sensorFreeAcceleration - G_gravity);

        % -----------Base 3D Angular Acc: base_angAcc
        IDcomparisonParams.baseAcc(blockIdx).baseAngAcc_wrtBase = G_R_base' * ...
            IDcomparisonParams.baseAcc(blockIdx).baseAngAcc_wrtG;
        
        % -----------Base 3D Angular Vel: base_angVel
        IDcomparisonParams.baseVel(blockIdx).baseAngVel_wrtBase = G_R_base' * ...
            IDcomparisonParams.baseVel(blockIdx).baseAngVel_wrtG;
    end
end

%% Save file
save(fullfile(bucket.pathToProcessedData,'IDcomparisonParams.mat'),'IDcomparisonParams');
