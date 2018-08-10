
% Plots EXO vs. NO EXO
cd data_folder;

%% Preliminaries
close all;

% bucket.datasetRoot = fullfile(pwd, 'dataJSI');
bucket.datasetRoot = pwd;

% Group options
group1 = false;
group2 = true;

if group1
    % GROUP 1
    subjectID = [1,3,5,7,9,11];
    taskID = [0, 1];
end

if group2
    % GROUP 2
    subjectID = [2,4,6,8,10,12];
    taskID = [0, 1, 2];
end

% Blocks
block.labels = {'block1'; ...
    'block2'; ...
    'block3'; ...
    'block4'; ...
    'block5'};
block.nrOfBlocks = size(block.labels,1);

%% Plots
for subjIdx = 1 : length(subjectID)
    pathToSubject = fullfile(bucket.datasetRoot, sprintf('S%02d',subjectID(subjIdx)));
    
    if group1
        pathToTask00 = fullfile(pathToSubject,sprintf('task%d',taskID(1)));
        pathToTask01 = fullfile(pathToSubject,sprintf('task%d',taskID(2)));
        
        pathToProcessedData00 = fullfile(pathToTask00,'processed');
        pathToProcessedData01 = fullfile(pathToTask01,'processed');
        
        exo00 = load(fullfile(pathToProcessedData00,'exo.mat'));
        CoC01 = load(fullfile(pathToProcessedData01,'CoC.mat'));
        
        %         % ------------------ do task 00 vs. 01 comparison -----------------
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        for blockIdx = 1 : block.nrOfBlocks
            % -------- RIGHT
            subplot (5,1,blockIdx)
            % Task 01 --> NO EXO
            plot1 = plot(CoC01.CoC(blockIdx).Rsho_tauFirst(1,:),'lineWidth',1.5);
            hold on;
            % Task 00 --> EXO
            plot2 = plot(exo00.exo(blockIdx).torqueDiff_right,'lineWidth',1.5);
            title(sprintf('Right Shoulder, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend([plot1,plot2],{'noexo_01','exo_00'},'Location','northeast');
            set(leg,'Interpreter','latex');
            
        end
        
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        for blockIdx = 1 : block.nrOfBlocks
            % -------- LEFT
            subplot (5,1,blockIdx)
            % Task 01 --> NO EXO
            plot1 = plot(CoC01.CoC(blockIdx).Lsho_tauFirst(1,:),'lineWidth',1.5);
            hold on;
            % Task 00 --> EXO
            plot2 = plot(exo00.exo(blockIdx).torqueDiff_left,'lineWidth',1.5);
            title(sprintf('Left Shoulder, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend([plot1,plot2],{'noexo_01','exo_00'},'Location','northeast');
            set(leg,'Interpreter','latex');
        end
    else
        pathToTask00 = fullfile(pathToSubject,sprintf('task%d',taskID(1)));
        pathToTask01 = fullfile(pathToSubject,sprintf('task%d',taskID(2)));
        pathToTask02 = fullfile(pathToSubject,sprintf('task%d',taskID(3)));
        
        pathToProcessedData00 = fullfile(pathToTask00,'processed');
        pathToProcessedData01 = fullfile(pathToTask01,'processed');
        pathToProcessedData02 = fullfile(pathToTask02,'processed');
        
        CoC00 = load(fullfile(pathToProcessedData00,'CoC.mat'));
        exo01 = load(fullfile(pathToProcessedData01,'exo.mat'));
        CoC02 = load(fullfile(pathToProcessedData02,'CoC.mat'));
        
        % ------------------ do task 01 vs. 00 comparison ----------------
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        for blockIdx = 1 : block.nrOfBlocks
            % -------- RIGHT
            subplot (5,1,blockIdx)
            % Task 00 --> NO EXO
            plot1 = plot(CoC00.CoC(blockIdx).Rsho_tauFirst(1,:),'lineWidth',1.5);
            hold on;
            % Task 01 --> EXO
            plot2 = plot(exo01.exo(blockIdx).torqueDiff_right,'lineWidth',1.5);
            title(sprintf('Right Shoulder, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend([plot1,plot2],{'noexo_00','exo_01'},'Location','northeast');
            set(leg,'Interpreter','latex');
            
        end
        
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        for blockIdx = 1 : block.nrOfBlocks
            % -------- LEFT
            subplot (5,1,blockIdx)
            % Task 00 --> NO EXO
            plot1 = plot(CoC00.CoC(blockIdx).Lsho_tauFirst(1,:),'lineWidth',1.5);
            hold on;
            % Task 01 --> EXO
            plot2 = plot(exo01.exo(blockIdx).torqueDiff_left,'lineWidth',1.5);
            title(sprintf('Left Shoulder, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend([plot1,plot2],{'noexo_00','exo_01'},'Location','northeast');
            set(leg,'Interpreter','latex');
        end
        
        % ------------------ do task 01 vs. 02 comparison -----------------
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        for blockIdx = 1 : block.nrOfBlocks
            % -------- RIGHT
            subplot (5,1,blockIdx)
            % Task 02 --> NO EXO
            plot1 = plot(CoC02.CoC(blockIdx).Rsho_tauFirst(1,:),'lineWidth',1.5);
            hold on;
            % Task 01 --> EXO
            plot2 = plot(exo01.exo(blockIdx).torqueDiff_right,'lineWidth',1.5);
            title(sprintf('Right Shoulder, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend([plot1,plot2],{'noexo_02','exo_01'},'Location','northeast');
            set(leg,'Interpreter','latex');
            
        end
        
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        for blockIdx = 1 : block.nrOfBlocks
            % -------- LEFT
            subplot (5,1,blockIdx)
            % Task 02 --> NO EXO
            plot1 = plot(CoC02.CoC(blockIdx).Lsho_tauFirst(1,:),'lineWidth',1.5);
            hold on;
            % Task 01 --> EXO
            plot2 = plot(exo01.exo(blockIdx).torqueDiff_left,'lineWidth',1.5);
            title(sprintf('Left Shoulder, S%02d, Block %s',subjectID(subjIdx), num2str(blockIdx)));
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend([plot1,plot2],{'noexo_02','exo_01'},'Location','northeast');
            set(leg,'Interpreter','latex');
        end
    end
end
