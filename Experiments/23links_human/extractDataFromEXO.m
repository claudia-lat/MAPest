
%% Preliminaries
close all;

% Load EXO data
EXO.dataFilename = fullfile(bucket.datasetRoot, 'EXOforceData.csv');

% Extract EXO data
EXO.extractedData = table2array(readtable(EXO.dataFilename,'Delimiter',';'));
EXO.extractedData_noHeader = str2double(EXO.extractedData(2:end,:));
EXO.subjTorqueID = (4:3:37);

% Load human kinematics
load(fullfile(bucket.pathToProcessedData,'synchroKin.mat'));
load(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'));

% Option for plots
EXO.opts.plots = true;

% Blocks
block.labels = {'block1'; ...
    'block2'; ...
    'block3'; ...
    'block4'; ...
    'block5'};
block.nrOfBlocks = size(block.labels,1);

%% Transform angles from CURRENT to FIXED frames
% IK data are expressed in current frames. To help this analisys is useful
% to have the shoulder angles expressed in fixed frame.

for sjIdx = 1 : size(selectedJoints,1)
    % Right shoulder
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_rotx'))
        EXO.jRshoRotx_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_roty'))
        EXO.jRshoRoty_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_rotz'))
        EXO.jRshoRotz_idx = sjIdx;
    end
    % Left Shoulder
    if (strcmp(selectedJoints{sjIdx,1},'jLeftShoulder_rotx'))
        EXO.jLshoRotx_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jLeftShoulder_roty'))
        EXO.jLshoRoty_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jLeftShoulder_rotz'))
        EXO.jLshoRotz_idx = sjIdx;
    end
end

for blockIdx = 1 : block.nrOfBlocks
    % -------Right shoulder
    EXO.jRightShoulder_rotx_grad = synchroKin(blockIdx).q(EXO.jRshoRotx_idx,:) * 180/pi; %deg
    EXO.jRightShoulder_roty_grad = synchroKin(blockIdx).q(EXO.jRshoRoty_idx,:) * 180/pi; %deg
    EXO.jRightShoulder_rotz_grad = synchroKin(blockIdx).q(EXO.jRshoRotz_idx,:) * 180/pi; %deg
    
    EXO.RightRpy_deg = zeros (3,size(EXO.jRightShoulder_rotx_grad,2));
    for i = 1 : size(EXO.jRightShoulder_rotx_grad,2)
        EXO.Rightq_shoulder = [synchroKin(blockIdx).q(EXO.jRshoRotx_idx,i); ...
            synchroKin(blockIdx).q(EXO.jRshoRoty_idx,i); ...
            synchroKin(blockIdx).q(EXO.jRshoRotz_idx,i)];
        [EXO.RightRx, EXO.RightRy, EXO.RightRz] = angle2rots(EXO.Rightq_shoulder);
        
        % The angles are expressed in current frame
        EXO.RightR_CF = EXO.RightRx * EXO.RightRy * EXO.RightRz;
        
        [EXO.RightRpy] = mat2RPY(EXO.RightR_CF);
        EXO.RightRpy_deg(:,i) = EXO.RightRpy * 180/pi;
    end
    
    % -------Left Shoulder
    EXO.jLeftShoulder_rotx_grad = synchroKin(blockIdx).q(EXO.jLshoRotx_idx,:) * 180/pi; %deg
    EXO.jLeftShoulder_roty_grad = synchroKin(blockIdx).q(EXO.jLshoRoty_idx,:) * 180/pi; %deg
    EXO.jLeftShoulder_rotz_grad = synchroKin(blockIdx).q(EXO.jLshoRotz_idx,:) * 180/pi; %deg

    EXO.LeftRpy_deg = zeros (3,size(EXO.jLeftShoulder_rotx_grad,2));
    for i = 1 : size(EXO.jLeftShoulder_rotx_grad,2)
        EXO.Leftq_shoulder = [synchroKin(blockIdx).q(EXO.jLshoRotx_idx,i); ...
            synchroKin(blockIdx).q(EXO.jLshoRoty_idx,i); ...
            synchroKin(blockIdx).q(EXO.jLshoRotz_idx,i)];
        [EXO.LeftRx, EXO.LeftRy, EXO.LeftRz] = angle2rots(EXO.Leftq_shoulder);
        
        % The angles are expressed in current frame
        EXO.LeftR_CF = EXO.LeftRx * EXO.LeftRy * EXO.LeftRz;
        
        [EXO.LeftRpy] = mat2RPY(EXO.LeftR_CF);
        EXO.LeftRpy_deg(:,i) = EXO.LeftRpy * 180/pi;
    end

    %% Match IK angles with the EXO angles
    % We need to extract the angles from the table and to match them with the
    % IK angles of the rightShoulder (already transformed in FIXED frame
    % formalism)

    % The shoulder_angle corresponds to the vector rotx EXO.rpy_deg(1,:)

    % -------Right shoulder
    EXO.qToCompare_right = (- EXO.RightRpy_deg(1,:) + 90)'; % operation to compare the angles: change sign and then +90 deg
    EXO.qToCompare_right_round = round(EXO.qToCompare_right);

    EXO.tau_EXO_right = zeros(1,size(EXO.qToCompare_right_round,1));
    for qIdx = 1 : size(EXO.qToCompare_right_round,1)
        for tableIdx = 1 : size(EXO.extractedData_noHeader,1)
            if (EXO.qToCompare_right_round(qIdx) == EXO.extractedData_noHeader(tableIdx,1))
                EXO.tau_EXO_right(qIdx) = EXO.extractedData_noHeader(tableIdx,EXO.subjTorqueID(subjectID));
            end
        end
    end
    % -------Left shoulder
    EXO.qToCompare_left = (EXO.LeftRpy_deg(1,:) + 90)'; % operation to compare the angles: +90 deg
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
    exo(blockIdx).block = block.labels(blockIdx);
    exo(blockIdx).torqueFromTable_right = EXO.tau_EXO_right;
    exo(blockIdx).torqueFromTable_left = EXO.tau_EXO_left;
    % tau_MAPest - tau_EXO
    exo(blockIdx).torqueDiff_right = estimatedVariables.tau(blockIdx).values(EXO.jRshoRotx_idx,:) + EXO.tau_EXO_right; % this is a sum because the axes rotation
    exo(blockIdx).torqueDiff_left = estimatedVariables.tau(blockIdx).values(EXO.jLshoRotx_idx,:) + EXO.tau_EXO_left; % no axes rotation
end

%% PLOTS section
if EXO.opts.plots
    % ------------Current VS fixed frame, Right Shoulder-------------------
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;

    blockIdx = 3;
    subplot (211)
    plot(EXO.jRightShoulder_rotx_grad,'r','Linewidth',1.5)
    hold on
    plot(EXO.jRightShoulder_roty_grad,'g','Linewidth',1.5)
    hold on
    plot(EXO.jRightShoulder_rotz_grad,'b','Linewidth',1.5)
    ylabel('joint angle [deg]','FontSize',15);
    % xlabel('samples','FontSize',15);
    title(sprintf('Right CURRENT FRAME, Block %s', num2str(blockIdx)))
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)

    subplot (212)
    plot(EXO.RightRpy_deg(1,:),'r','Linewidth',1.5)
    hold on
    plot(EXO.RightRpy_deg(2,:),'g','Linewidth',1.5)
    hold on
    plot(EXO.RightRpy_deg(3,:),'b','Linewidth',1.5)
    ylabel('joint angle [deg]','FontSize',15);
    xlabel('samples','FontSize',15);
    title(sprintf('Right  FIXED FRAME, Block %s', num2str(blockIdx)))
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)

    % ------------Current VS fixed frame, Left Shoulder--------------------
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');

    blockIdx = 3;
    subplot (211)
    plot(EXO.jLeftShoulder_rotx_grad,'r','Linewidth',1.5)
    hold on
    plot(EXO.jLeftShoulder_roty_grad,'g','Linewidth',1.5)
    hold on
    plot(EXO.jLeftShoulder_rotz_grad,'b','Linewidth',1.5)
    ylabel('joint angle [deg]','FontSize',15);
    % xlabel('samples','FontSize',15);
    title(sprintf('Left CURRENT FRAME, Block %s', num2str(blockIdx)))
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)

    subplot (212)
    plot(EXO.LeftRpy_deg(1,:),'r','Linewidth',1.5)
    hold on
    plot(EXO.LeftRpy_deg(2,:),'g','Linewidth',1.5)
    hold on
    plot(EXO.LeftRpy_deg(3,:),'b','Linewidth',1.5)
    ylabel('joint angle [deg]','FontSize',15);
    xlabel('samples','FontSize',15);
    title(sprintf('Left FIXED FRAME, Block %s', num2str(blockIdx)))
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)

    % ------------Final torque, Right Shoulder-----------------------------
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;

    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % tau MAPest without any exo consideration
        plot1 = plot(estimatedVariables.tau(blockIdx).values(EXO.jRshoRotx_idx,:),'lineWidth',1.5);
        hold on;
        % tau from EXO tab
        plot2 = plot(exo(blockIdx).torqueFromTable_right,'lineWidth',1.5);
        hold on;
        % tau_diff
        plot3 = plot(exo(blockIdx).torqueDiff_right,'lineWidth',1.5);
        title(sprintf('Right, Block %s', num2str(blockIdx)));
        ylabel('torque');
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2,plot3],{'$\tau_{MAPest}$','$\tau_{EXO}$','$\tau_{diff}$'},'Location','northeast');
        set(leg,'Interpreter','latex');
    end
    % ------------Final torque, Left Shoulder------------------------------
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;

    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % tau MAPest without any exo consideration
        plot1 = plot(estimatedVariables.tau(blockIdx).values(EXO.jLshoRotx_idx,:),'lineWidth',1.5);
        hold on;
        % tau from EXO tab
        plot2 = plot(exo(blockIdx).torqueFromTable_left,'lineWidth',1.5);
        hold on;
        % tau_diff
        plot3 = plot(exo(blockIdx).torqueDiff_left,'lineWidth',1.5);
        title(sprintf('Left, Block %s', num2str(blockIdx)));
        ylabel('torque');
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2,plot3],{'$\tau_{MAPest}$','$\tau_{EXO}$','$\tau_{diff}$'},'Location','northeast');
        set(leg,'Interpreter','latex');
    end
end




%% Utility
function [Rx, Ry, Rz] = angle2rots(x)
%ANGLE2ROTS computes the three rotation matrices given a vector x of angle 3x1
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
