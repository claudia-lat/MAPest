
% Plots EXO vs. NO EXO

%% Preliminaries
close all;
clc; clear all;
addpath(genpath('../../external'));

bucket.datasetRoot = fullfile(pwd, 'dataJSI');
% bucket.datasetRoot = pwd;

% Group options
group1 = true;
group2 = false;

if group1
    % GROUP 1
    subjectID = [1];
    % subjectID = [1,3,5,7,9,11];
    taskID = [0,1];
end
if group2
    % GROUP 2
    subjectID = [2,4,6,8,10,12];
    taskID = [0, 1, 2];
end

% Blocksclod
block.labels = {'block1'; ...
    'block2'; ...
    'block3'; ...
    'block4'; ...
    'block5'};
block.nrOfBlocks = size(block.labels,1);

% NO EXO color
orangeAnDycolor = [0.952941176470588   0.592156862745098   0.172549019607843];
% WITH EXO color
greenAnDycolor  = [0.282352941176471   0.486274509803922   0.427450980392157];

%% Plots
for subjIdx = 1 : length(subjectID)
    pathToSubject = fullfile(bucket.datasetRoot, sprintf('S%02d',subjectID(subjIdx)));
    
    if group1
        pathToTask00 = fullfile(pathToSubject,sprintf('task%d',taskID(1)));
        pathToTask01 = fullfile(pathToSubject,sprintf('task%d',taskID(2)));
        
        pathToProcessedData00 = fullfile(pathToTask00,'processed');
        pathToProcessedData01 = fullfile(pathToTask01,'processed');
        
        exo00   = load(fullfile(pathToProcessedData00,'exo_insideMAP.mat'));
        noexo01 = load(fullfile(pathToProcessedData01,'estimatedVariables.mat'));
        
        selectedJoints = load(fullfile(pathToProcessedData01,'selectedJoints.mat'));
        
        % Create a struct .mat for the comparison with NE and WE per subject
        comparisonWEvsNE.jointsOrder = selectedJoints.selectedJoints;
        comparisonWEvsNE.torqueNorm = struct;
        
        % ------------------ do task 01 vs. 00 comparison -----------------
%         for jointsIdx = 1 : length(selectedJoints.selectedJoints)
%             fig = figure('Name', 'EXO vs NOEXO analysis','NumberTitle','off');
%             axes1 = axes('Parent',fig,'FontSize',16);
%             box(axes1,'on');
%             hold(axes1,'on');
%             grid on;
%             
%             for blockIdx = 1 : block.nrOfBlocks
%                 subplot (5,1,blockIdx)
%                 % Task 01 --> NO EXO
%                 plot1 = plot(noexo01.estimatedVariables.tau(blockIdx).values(jointsIdx,:),'color',orangeAnDycolor,'lineWidth',1.5);
%                 hold on;
%                 % Task 00 --> WITH EXO
%                 plot2 = plot(exo00.exo_insideMAP.estimatedVariables.tau(blockIdx).values(jointsIdx,:),'color',greenAnDycolor,'lineWidth',1.5);
%                 title(sprintf('%s, S%02d, Block %s',selectedJoints.selectedJoints{jointsIdx,1}, ...
%                     subjectID(subjIdx), num2str(blockIdx)),'Interpreter','latex');
%                 ylabel('torque [Nm]');
%                 if blockIdx == 5
%                     xlabel('samples');
%                 end
%                 set(gca,'FontSize',15)
%                 grid on;
%                 %legend
%                 leg = legend([plot1,plot2],{'NE','WE'},'Location','northeast');
%                 set(leg,'Interpreter','latex');
%                 axis tight
%             end
%         end
    else
        pathToTask00 = fullfile(pathToSubject,sprintf('task%d',taskID(1)));
        pathToTask01 = fullfile(pathToSubject,sprintf('task%d',taskID(2)));
        pathToTask02 = fullfile(pathToSubject,sprintf('task%d',taskID(3)));
        
        pathToProcessedData00 = fullfile(pathToTask00,'processed');
        pathToProcessedData01 = fullfile(pathToTask01,'processed');
        pathToProcessedData02 = fullfile(pathToTask02,'processed');
        
        noexo00 = load(fullfile(pathToProcessedData00,'estimatedVariables.mat'));
        exo01   = load(fullfile(pathToProcessedData01,'exo_insideMAP.mat'));
        noexo02 = load(fullfile(pathToProcessedData02,'estimatedVariables.mat'));
        
        % ------------------ do task 02 vs. 01 comparison -----------------
        for jointsIdx = 1 : length(selectedJoints.selectedJoints)
            fig = figure('Name', 'EXO vs NOEXO analysis','NumberTitle','off');
            axes1 = axes('Parent',fig,'FontSize',16);
            box(axes1,'on');
            hold(axes1,'on');
            grid on;
            for blockIdx = 1 : block.nrOfBlocks
                subplot (5,1,blockIdx)
                % Task 02 --> NO EXO
                plot1 = plot(noexo02.estimatedVariables.tau(blockIdx).values(jointsIdx,:),'color',orangeAnDycolor,'lineWidth',1.5);
                hold on;
                % Task 01 --> WITH EXO
                plot2 = plot(exo01.exo_insideMAP.estimatedVariables.tau(blockIdx).values(jointsIdx,:),greenAnDycolor,'lineWidth''lineWidth',1.5);
                title(sprintf('%s, S%02d, Block %s',selectedJoints.selectedJoints{jointsIdx,1}, ...
                    subjectID(subjIdx), num2str(blockIdx)),'Interpreter','latex');
                ylabel('torque [Nm]');
                if blockIdx == 5
                    xlabel('samples');
                end
                set(gca,'FontSize',15)
                grid on;
                %legend
                leg = legend([plot1,plot2],{'NE','WE'},'Location','northeast');
                set(leg,'Interpreter','latex');
                axis tight
            end
        end
    end
end

%% Norm computation
torso_range    = (1:14);
rightArm_range = (15:22);
leftArm_range  = (23:30);
rightLeg_range = (31:39);
leftLeg_range  = (40:48);

if group1
    % exo00
    for blockIdx = 1 : block.nrOfBlocks
        comparisonWEvsNE.torqueNorm(blockIdx).block = block.labels(blockIdx);
        len = size(exo00.exo_insideMAP.estimatedVariables.tau(1).values,2);
        for i = 1 : len
            comparisonWEvsNE.torqueNorm(blockIdx).complete_exo00(1,i) = norm(exo00.exo_insideMAP.estimatedVariables.tau(blockIdx).values(:,i));
            comparisonWEvsNE.torqueNorm(blockIdx).torso_exo00(1,i)    = norm(exo00.exo_insideMAP.estimatedVariables.tau(blockIdx).values(torso_range,i));
            comparisonWEvsNE.torqueNorm(blockIdx).rightArm_exo00(1,i) = norm(exo00.exo_insideMAP.estimatedVariables.tau(blockIdx).values(rightArm_range,i));
            comparisonWEvsNE.torqueNorm(blockIdx).leftArm_exo00(1,i)  = norm(exo00.exo_insideMAP.estimatedVariables.tau(blockIdx).values(leftArm_range,i));
            comparisonWEvsNE.torqueNorm(blockIdx).rightLeg_exo00(1,i) = norm(exo00.exo_insideMAP.estimatedVariables.tau(blockIdx).values(rightLeg_range,i));
            comparisonWEvsNE.torqueNorm(blockIdx).leftLeg_exo00(1,i)  = norm(exo00.exo_insideMAP.estimatedVariables.tau(blockIdx).values(leftLeg_range,i));
        end
    end
    
    % noexo01
    for blockIdx = 1 : block.nrOfBlocks
        len = size(noexo01.estimatedVariables.tau(1).values,2);
        for i = 1 : len
            comparisonWEvsNE.torqueNorm(blockIdx).complete_noexo01(1,i) = norm(noexo01.estimatedVariables.tau(blockIdx).values(:,i));
            comparisonWEvsNE.torqueNorm(blockIdx).torso_noexo01(1,i)    = norm(noexo01.estimatedVariables.tau(blockIdx).values(torso_range,i));
            comparisonWEvsNE.torqueNorm(blockIdx).rightArm_noexo01(1,i) = norm(noexo01.estimatedVariables.tau(blockIdx).values(rightArm_range,i));
            comparisonWEvsNE.torqueNorm(blockIdx).leftArm_noexo01(1,i)  = norm(noexo01.estimatedVariables.tau(blockIdx).values(leftArm_range,i));
            comparisonWEvsNE.torqueNorm(blockIdx).rightLeg_noexo01(1,i) = norm(noexo01.estimatedVariables.tau(blockIdx).values(rightLeg_range,i));
            comparisonWEvsNE.torqueNorm(blockIdx).leftLeg_noexo01(1,i)  = norm(noexo01.estimatedVariables.tau(blockIdx).values(leftLeg_range,i));
        end
    end
    
    %% Norm plots
    
    % ============= general
    fig = figure('Name', 'EXO vs NOEXO norm COMPLETE','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % Task 00 --> NO EXO
        plot1 = plot(comparisonWEvsNE.torqueNorm(blockIdx).complete_noexo01,'color',orangeAnDycolor,'lineWidth',1.5);
        hold on;
        % Task 01 --> WITH EXO
        plot2 = plot(comparisonWEvsNE.torqueNorm(blockIdx).complete_exo00,'color',greenAnDycolor,'lineWidth',1.5);
        title(sprintf('Complete norm, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
        ylabel('\tau norm');
        if blockIdx == 5
            xlabel('samples');
        end
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2],{'NE','WE'},'Location','northeast');
        set(leg,'Interpreter','latex');
        axis tight
    end
    
    % ============= torso
    fig = figure('Name', 'EXO vs NOEXO norm TORSO','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % Task 00 --> NO EXO
        plot1 = plot(comparisonWEvsNE.torqueNorm(blockIdx).torso_noexo01,'color',orangeAnDycolor,'lineWidth',1.5);
        hold on;
        % Task 01 --> WITH EXO
        plot2 = plot(comparisonWEvsNE.torqueNorm(blockIdx).torso_exo00,'color',greenAnDycolor,'lineWidth',1.5);
        title(sprintf('Torso norm, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
        ylabel('\tau norm');
        if blockIdx == 5
            xlabel('samples');
        end
        ylim([0 200]);
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2],{'NE','WE'},'Location','northeast');
        set(leg,'Interpreter','latex');
        axis tight
    end
    
    % ============= right arm
    fig = figure('Name', 'EXO vs NOEXO norm RIGHT ARM','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % Task 00 --> NO EXO
        plot1 = plot(comparisonWEvsNE.torqueNorm(blockIdx).rightArm_noexo01,'color',orangeAnDycolor,'lineWidth',1.5);
        hold on;
        % Task 01 --> WITH EXO
        plot2 = plot(comparisonWEvsNE.torqueNorm(blockIdx).rightArm_exo00,'color',greenAnDycolor,'lineWidth',1.5);
        title(sprintf('Right arm norm, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
        ylabel('\tau norm');
        if blockIdx == 5
            xlabel('samples');
        end
        ylim([0 200]);
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2],{'NE','WE'},'Location','northeast');
        set(leg,'Interpreter','latex');
        axis tight
    end
    
    % ============= left arm
    fig = figure('Name', 'EXO vs NOEXO norm LEFT ARM','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % Task 00 --> NO EXO
        plot1 = plot(comparisonWEvsNE.torqueNorm(blockIdx).leftArm_noexo01,'color',orangeAnDycolor,'lineWidth',1.5);
        hold on;
        % Task 01 --> WITH EXO
        plot2 = plot(comparisonWEvsNE.torqueNorm(blockIdx).leftArm_exo00,'color',greenAnDycolor,'lineWidth',1.5);
        title(sprintf('Left arm norm, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
        ylabel('\tau norm');
        if blockIdx == 5
            xlabel('samples');
        end
        ylim([0 200]);
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2],{'NE','WE'},'Location','northeast');
        set(leg,'Interpreter','latex');
        axis tight
    end
    tightfig;
    
    % ============= right leg
    fig = figure('Name', 'EXO vs NOEXO norm_RIGHT LEG','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % Task 00 --> NO EXO
        plot1 = plot(comparisonWEvsNE.torqueNorm(blockIdx).rightLeg_noexo01,'color',orangeAnDycolor,'lineWidth',1.5);
        hold on;
        % Task 01 --> WITH EXO
        plot2 = plot(comparisonWEvsNE.torqueNorm(blockIdx).rightLeg_exo00,'color',greenAnDycolor,'lineWidth',1.5);
        title(sprintf('Right leg norm, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
        ylabel('\tau norm');
        if blockIdx == 5
            xlabel('samples');
        end
        ylim([0 200]);
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2],{'NE','WE'},'Location','northeast');
        set(leg,'Interpreter','latex');
        axis tight
    end
    
    % ============= left leg
    fig = figure('Name', 'EXO vs NOEXO norm_LEFT LEG','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % Task 00 --> NO EXO
        plot1 = plot(comparisonWEvsNE.torqueNorm(blockIdx).leftLeg_noexo01,'color',orangeAnDycolor,'lineWidth',1.5);
        hold on;
        % Task 01 --> WITH EXO
        plot2 = plot(comparisonWEvsNE.torqueNorm(blockIdx).leftLeg_exo00,'color',greenAnDycolor,'lineWidth',1.5);
        title(sprintf('Left leg norm, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
        ylabel('\tau norm');
        if blockIdx == 5
            xlabel('samples');
        end
        ylim([0 200]);
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2],{'NE','WE'},'Location','northeast');
        set(leg,'Interpreter','latex');
        axis tight
    end
end
