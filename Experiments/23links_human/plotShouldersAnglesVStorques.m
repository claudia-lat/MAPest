
%% Preliminaries
close all;

% Load human kinematics
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

%% Index of the shoulder joints
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

%% Plots of shoulders joint angles VS torques
% The angles (and therefore the torques) are expressed in CURRENT FRAME
% (terna mobile).
for blockIdx = 1 : block.nrOfBlocks
    len_block = size(synchroKin(blockIdx).q,2);
    
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    
    % ------jRightShoulder
    subplot (221)
    plot(synchroKin(blockIdx).q(jRshoRotx_idx,:)* 180/pi,'r','Linewidth',1.5)
    hold on
    plot(synchroKin(blockIdx).q(jRshoRoty_idx,:)* 180/pi,'g','Linewidth',1.5)
    hold on
    plot(synchroKin(blockIdx).q(jRshoRotz_idx,:)* 180/pi,'b','Linewidth',1.5)
    ylabel('joint angle [deg]','FontSize',18);
    xlabel('samples','FontSize',18);
    title(sprintf('jRightShoulder, Block %s', num2str(blockIdx)),'FontSize',15)
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)
    
    subplot (223)
    plot(estimatedVariables.tau(blockIdx).values(jRshoRotx_idx,:),'r','Linewidth',1.5)
    hold on
    plot(estimatedVariables.tau(blockIdx).values(jRshoRoty_idx,:),'g','Linewidth',1.5)
    hold on
    plot(estimatedVariables.tau(blockIdx).values(jRshoRotz_idx,:),'b','Linewidth',1.5)
    ylabel('torque [Nm]','FontSize',18);
    xlabel('samples','FontSize',18);
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)
    
    % ------jLeftShoulder
    subplot (222)
    plot(synchroKin(blockIdx).q(jLshoRotx_idx,:)* 180/pi,'r','Linewidth',1.5)
    hold on
    plot(synchroKin(blockIdx).q(jLshoRoty_idx,:)* 180/pi,'g','Linewidth',1.5)
    hold on
    plot(synchroKin(blockIdx).q(jLshoRotz_idx,:)* 180/pi,'b','Linewidth',1.5)
    ylabel('joint angle [deg]','FontSize',18);
    xlabel('samples','FontSize',18);
    title(sprintf('jLeftShoulder, Block %s', num2str(blockIdx)),'FontSize',15);
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)
    
    subplot (224)
    plot(estimatedVariables.tau(blockIdx).values(jLshoRotx_idx,:),'r','Linewidth',1.5)
    hold on
    plot(estimatedVariables.tau(blockIdx).values(jLshoRoty_idx,:),'g','Linewidth',1.5)
    hold on
    plot(estimatedVariables.tau(blockIdx).values(jLshoRotz_idx,:),'b','Linewidth',1.5)
    ylabel('torque [Nm]','FontSize',18);
    xlabel('samples','FontSize',18);
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)
end
