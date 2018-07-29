
% Comparison MAPest torques with EXO torques

%% Preliminaries
close all;

% Load variables
load(fullfile(bucket.pathToProcessedData,'synchroKin.mat'));
load(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'));
load(fullfile(bucket.pathToProcessedData,'estimatedVariables.mat'));

% Blocks
block.labels = {'block1'; ...
    'block2'; ...
    'block3'; ...
    'block4'; ...
    'block5'};
block.nrOfBlocks = size(block.labels,1);

for sjIdx = 1 : size(selectedJoints,1)
    % Right shoulder
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_rotx'))
        jRshoRotx_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_roty'))
        jRshoRoty_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_rotz'))
        jRshoRotz_idx = sjIdx;
    end
    % Left Shoulder
    if (strcmp(selectedJoints{sjIdx,1},'jLeftShoulder_rotx'))
        jLshoRotx_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jLeftShoulder_roty'))
        jLshoRoty_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jLeftShoulder_rotz'))
        jLshoRotz_idx = sjIdx;
    end
end

%% ======================= CHANGE OF COORDINATES ==========================
for blockIdx = 1 : block.nrOfBlocks
    len = size(synchroKin(blockIdx).masterTime ,2);
    
    %% -------Right shoulder angles/torques analysis
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    
    % Original angles from Opensim IK (q) in deg
    qx_rightSho = synchroKin(blockIdx).q(jRshoRotx_idx,:) * 180/pi; %deg
    qy_rightSho = synchroKin(blockIdx).q(jRshoRoty_idx,:) * 180/pi; %deg
    qz_rightSho = synchroKin(blockIdx).q(jRshoRotz_idx,:) * 180/pi; %deg
    
    % Torques estimated by MAP with angles q
    tau_rightSho = [estimatedVariables.tau(blockIdx).values(jRshoRotx_idx,:); ...
        estimatedVariables.tau(blockIdx).values(jRshoRoty_idx,:); ...
        estimatedVariables.tau(blockIdx).values(jRshoRotz_idx,:)];
    
    subplot (421) % angles q
    plot(qx_rightSho,'r','Linewidth',1.5)
    hold on
    plot(qy_rightSho,'g','Linewidth',1.5)
    hold on
    plot(qz_rightSho,'b','Linewidth',1.5)
    ylabel('$q$ [deg]','FontSize',15,'Interpreter','latex');
    xlabel('samples','FontSize',15);
    title(sprintf('jRightShoulder, Block %s', num2str(blockIdx)))
    leg = legend('$q_x$','$q_y$','$q_z$');
    set(leg,'FontSize',17)
    set(leg,'Interpreter','latex');
    
    subplot (423) %torques tau
    plot(tau_rightSho(1,:),'r','Linewidth',1.5)
    hold on
    plot(tau_rightSho(2,:),'g','Linewidth',1.5)
    hold on
    plot(tau_rightSho(3,:),'b','Linewidth',1.5)
    ylabel('$\tau$ [Nm]','FontSize',15,'Interpreter','latex');
    xlabel('samples','FontSize',15);
    leg = legend('$\tau_x$','$\tau_y$','$\tau_z$');
    set(leg,'FontSize',17)
    set(leg,'Interpreter','latex');
    
    qFirst_rightSho = zeros(3,len);
    jacobian_q_rightSho = cell(len,1);
    tauFirst_rightSho = zeros(3,len);
    for i = 1 : len
        % They are expressed in current frame (terna mobile) and the order of
        % consecutive rotations is: R_q = R_qx * R_qy * R_qz (from models)
        q_rightSho = [synchroKin(blockIdx).q(jRshoRotx_idx,i); ...
            synchroKin(blockIdx).q(jRshoRoty_idx,i); ...
            synchroKin(blockIdx).q(jRshoRotz_idx,i)];
        [R_qx_rightSho, R_qy_rightSho, R_qz_rightSho] = angle2rots(q_rightSho);
        R_q_rightSho = R_qx_rightSho * R_qy_rightSho * R_qz_rightSho;
        
        % Compute angles(qFirst) with the coordinates change
        qxFirst_rightSho = atan(R_q_rightSho(3,2)/sqrt(R_q_rightSho(1,2)^2 + R_q_rightSho(2,2)^2));
        qyFirst_rightSho = atan(-R_q_rightSho(3,1)/R_q_rightSho(3,3));
        qzFirst_rightSho = atan(-R_q_rightSho(1,2)/R_q_rightSho(2,2));
        qFirst_rightSho(:,i) = [qxFirst_rightSho; qyFirst_rightSho; qzFirst_rightSho] * 180/pi; %deg;
        
        % Compute the Jacobian J_q
        a = 1/(1 + (R_q_rightSho(3,2)/sqrt(R_q_rightSho(1,2)^2 + R_q_rightSho(2,2)^2))^2);
        b = 1/(1 + (-R_q_rightSho(3,1)/R_q_rightSho(3,3))^2);
        c = 1/(1 + (-R_q_rightSho(1,2)/R_q_rightSho(2,2))^2);
        
        jacobian_q_rightSho{i} = [ a, 0, 0;
            0, b, 0;
            0, 0, c];
        
        % Compute the new torque (tauFirst) associated to the angle qFirst
        % (procedure obtained by applying D'Alambert principle)
        tauFirst_rightSho(:,i) = inv(jacobian_q_rightSho{i}') * tau_rightSho(:,i);
    end
    
    subplot (425) %angles qFirst
    plot(qFirst_rightSho(1,:),'r','Linewidth',1.5)
    hold on
    plot(qFirst_rightSho(2,:),'g','Linewidth',1.5)
    hold on
    plot(qFirst_rightSho(3,:),'b','Linewidth',1.5)
    ylabel('$q\prime$ [deg]','FontSize',15,'Interpreter','latex');
    xlabel('samples','FontSize',15);
%     title(sprintf('jRightShoulder, Block %s', num2str(blockIdx)))
    leg = legend('${q_x}\prime$','${q_y}\prime$','${q_z}\prime$');
    set(leg,'FontSize',17)
    set(leg,'Interpreter','latex');
    
    subplot (427) %torques tauFirst
    plot(tauFirst_rightSho(1,:),'r','Linewidth',1.5)
    hold on
    plot(tauFirst_rightSho(2,:),'g','Linewidth',1.5)
    hold on
    plot(tauFirst_rightSho(3,:),'b','Linewidth',1.5)
    ylabel('$\tau\prime$ [Nm]','FontSize',15,'Interpreter','latex');
    xlabel('samples','FontSize',15);
    leg = legend('${\tau_x}\prime$','${\tau_y}\prime$','${\tau_z}\prime$');
    set(leg,'FontSize',17)
    set(leg,'Interpreter','latex');
    
    
    %% -------Left shoulder angles/torques analysis
    
    % Original angles from Opensim IK (q) in deg
    qx_leftSho = synchroKin(blockIdx).q(jLshoRotx_idx,:) * 180/pi; %deg
    qy_leftSho = synchroKin(blockIdx).q(jLshoRoty_idx,:) * 180/pi; %deg
    qz_leftSho = synchroKin(blockIdx).q(jLshoRotz_idx,:) * 180/pi; %deg
    
    % Torques estimated by MAP with angles q
    tau_leftSho = [estimatedVariables.tau(blockIdx).values(jLshoRotx_idx,:); ...
        estimatedVariables.tau(blockIdx).values(jLshoRoty_idx,:); ...
        estimatedVariables.tau(blockIdx).values(jLshoRotz_idx,:)];
    
    subplot (422) % angles q
    plot(qx_leftSho,'r','Linewidth',1.5)
    hold on
    plot(qy_leftSho,'g','Linewidth',1.5)
    hold on
    plot(qz_leftSho,'b','Linewidth',1.5)
    ylabel('$q$ [deg]','FontSize',15,'Interpreter','latex');
    xlabel('samples','FontSize',15);
    title(sprintf('jLeftShoulder, Block %s', num2str(blockIdx)))
    leg = legend('$q_x$','$q_y$','$q_z$');
    set(leg,'FontSize',17)
    set(leg,'Interpreter','latex');
    
    subplot (424) %torques tau
    plot(tau_leftSho(1,:),'r','Linewidth',1.5)
    hold on
    plot(tau_leftSho(2,:),'g','Linewidth',1.5)
    hold on
    plot(tau_leftSho(3,:),'b','Linewidth',1.5)
    ylabel('$\tau$ [Nm]','FontSize',15,'Interpreter','latex');
    xlabel('samples','FontSize',15);
    leg = legend('$\tau_x$','$\tau_y$','$\tau_z$');
    set(leg,'FontSize',17)
    set(leg,'Interpreter','latex');
    
    qFirst_leftSho = zeros(3,len);
    jacobian_q_leftSho = cell(len,1);
    tauFirst_leftSho = zeros(3,len);
    for i = 1 : len
        % They are expressed in current frame (terna mobile) and the order of
        % consecutive rotations is: R_q = R_qx * R_qy * R_qz (from models)
        q_leftSho = [synchroKin(blockIdx).q(jLshoRotx_idx,i); ...
            synchroKin(blockIdx).q(jLshoRoty_idx,i); ...
            synchroKin(blockIdx).q(jLshoRotz_idx,i)];
        [R_qx_leftSho, R_qy_leftSho, R_qz_leftSho] = angle2rots(q_leftSho);
        R_q_leftSho = R_qx_leftSho * R_qy_leftSho * R_qz_leftSho;
        
        % Compute angles(qFirst) with the coordinates change
        qxFirst_leftSho = atan(R_q_leftSho(3,2)/sqrt(R_q_leftSho(1,2)^2 + R_q_leftSho(2,2)^2));
        qyFirst_leftSho = atan(-R_q_leftSho(3,1)/R_q_leftSho(3,3));
        qzFirst_leftSho = atan(-R_q_leftSho(1,2)/R_q_leftSho(2,2));
        qFirst_leftSho(:,i) = [qxFirst_leftSho; qyFirst_leftSho; qzFirst_leftSho] * 180/pi; %deg;
        
        % Compute the Jacobian J_q
        a = 1/(1 + (R_q_leftSho(3,2)/sqrt(R_q_leftSho(1,2)^2 + R_q_leftSho(2,2)^2))^2);
        b = 1/(1 + (-R_q_leftSho(3,1)/R_q_leftSho(3,3))^2);
        c = 1/(1 + (-R_q_leftSho(1,2)/R_q_leftSho(2,2))^2);
        
        jacobian_q_leftSho{i} = [ a, 0, 0;
            0, b, 0;
            0, 0, c];
        
        % Compute the new torque (tauFirst) associated to the angle qFirst
        % (procedure obtained by applying D'Alambert principle)
        tauFirst_leftSho(:,i) = inv(jacobian_q_leftSho{i}') * tau_leftSho(:,i);
    end
    
    subplot (426) %angles qFirst
    plot(qFirst_leftSho(1,:),'r','Linewidth',1.5)
    hold on
    plot(qFirst_leftSho(2,:),'g','Linewidth',1.5)
    hold on
    plot(qFirst_leftSho(3,:),'b','Linewidth',1.5)
    ylabel('$q\prime$ [deg]','FontSize',15,'Interpreter','latex');
    xlabel('samples','FontSize',15);
%     title(sprintf('jLeftShoulder, Block %s', num2str(blockIdx)))
    leg = legend('${q_x}\prime$','${q_y}\prime$','${q_z}\prime$');
    set(leg,'FontSize',17)
    set(leg,'Interpreter','latex');
    
    subplot (428) %torques tauFirst
    plot(tauFirst_leftSho(1,:),'r','Linewidth',1.5)
    hold on
    plot(tauFirst_leftSho(2,:),'g','Linewidth',1.5)
    hold on
    plot(tauFirst_leftSho(3,:),'b','Linewidth',1.5)
    ylabel('$\tau\prime$ [Nm]','FontSize',15,'Interpreter','latex');
    xlabel('samples','FontSize',15);
    leg = legend('${\tau_x}\prime$','${\tau_y}\prime$','${\tau_z}\prime$');
    set(leg,'FontSize',17)
    set(leg,'Interpreter','latex');
    
    %% Save comparison data in a struct
    exo(blockIdx).block = block.labels(blockIdx);
    exo(blockIdx).masterTime = synchroData(blockIdx).masterTime;
    % right shoulder
    exo(blockIdx).Rsho_q =[qx_rightSho; qy_rightSho; qz_rightSho];
    exo(blockIdx).Rsho_tau = tau_rightSho;
    exo(blockIdx).Rsho_qFirst = qFirst_rightSho;
    exo(blockIdx).Rsho_tauFirst = tauFirst_rightSho;
    % left shoulder
    exo(blockIdx).Lsho_q =[qx_leftSho; qy_leftSho; qz_leftSho];
    exo(blockIdx).Lsho_tau = tau_leftSho;
    exo(blockIdx).Lsho_qFirst = qFirst_leftSho;
    exo(blockIdx).Lsho_tauFirst = tauFirst_leftSho;
end

%% =========================== EXO ANALYSIS ===============================
% Extract data from EXO table and compare with the tauFirst(1,:).
% Note: the exo angles has to be compared with the tauFirst x and not 
% from tau_MAP anymore.
EXO.opts.plots = true;

% Load EXO data
EXO.dataFilename = fullfile(bucket.datasetRoot, 'EXOforceData.csv');

% Extract EXO data
EXO.extractedData = table2array(readtable(EXO.dataFilename,'Delimiter',';'));
EXO.extractedData_noHeader = str2double(EXO.extractedData(2:end,:));
EXO.subjTorqueID = (4:3:37);

for blockIdx = 1 : block.nrOfBlocks
    %% -------Right shoulder
    EXO.qToCompare_right = (- exo(blockIdx).Rsho_qFirst(1,:) + 90)'; % operation to compare the angles: change sign and then +90 deg
    EXO.qToCompare_right_round = round(EXO.qToCompare_right);
    
    EXO.tau_EXO_right = zeros(1,size(EXO.qToCompare_right_round,1));
    for qIdx = 1 : size(EXO.qToCompare_right_round,1)
        for tableIdx = 1 : size(EXO.extractedData_noHeader,1)
            if (EXO.qToCompare_right_round(qIdx) == EXO.extractedData_noHeader(tableIdx,1))
                EXO.tau_EXO_right(qIdx) = EXO.extractedData_noHeader(tableIdx,EXO.subjTorqueID(subjectID));
            end
        end
    end
    
    %% -------Left shoulder
    EXO.qToCompare_left = (exo(blockIdx).Lsho_qFirst(1,:) + 90)'; % operation to compare the angles: +90 deg
    EXO.qToCompare_left_round = round(EXO.qToCompare_left);
    
    EXO.tau_EXO_left = zeros(1,size(EXO.qToCompare_left_round,1));
    for qIdx = 1 : size(EXO.qToCompare_left_round,1)
        for tableIdx = 1 : size(EXO.extractedData_noHeader,1)
            if (EXO.qToCompare_left_round(qIdx) == EXO.extractedData_noHeader(tableIdx,1))
                EXO.tau_EXO_left(qIdx) = EXO.extractedData_noHeader(tableIdx,EXO.subjTorqueID(subjectID));
            end
        end
    end
    
    %% Save EXO torques in a struct  
    exo(blockIdx).torqueFromTable_right = EXO.tau_EXO_right;
    exo(blockIdx).torqueFromTable_left  = EXO.tau_EXO_left;
    % tauFirst_MAPest - tau_EXOfromTable
    exo(blockIdx).torqueDiff_right = exo(blockIdx).Rsho_tauFirst(1,:) - EXO.tau_EXO_right;
    exo(blockIdx).torqueDiff_left  = exo(blockIdx).Lsho_tauFirst(1,:) - EXO.tau_EXO_left;
end

%% Plots of (tauFirst_MAPest - tau_EXOfromTable)
if EXO.opts.plots
    EXOplots;
end

%% Utility
function [Rx, Ry, Rz] = angle2rots(x)
%ANGLE2ROTS computes the three rotation matrices given a vector x of angles
% 3x1 expressed in radians.
% It is required that the vector x is ordered as follow:
%    | angle of the rotation around x |
% x =| angle of the rotation around y |
%    | angle of the rotation around z |

Rx = [ 1     0          0    ;
       0 cos(x(1)) -sin(x(1));
       0 sin(x(1))  cos(x(1))];

Ry = [ cos(x(2)) 0 sin(x(2));
           0     1     0    ;
      -sin(x(2)) 0 cos(x(2))];

Rz = [ cos(x(3)) -sin(x(3)) 0;
       sin(x(3))  cos(x(3)) 0;
           0          0     1];
end
