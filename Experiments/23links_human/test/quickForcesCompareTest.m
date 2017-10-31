
% This script is for a quick test on the shoes and foreplates raw data.
% It loads the data from Subject/Trials and plots the forces on z axis for 
% a quick first insight.

clear;clc;close all;
addpath(genpath('../src')); 
pathToFolder = ('/Users/clatella/Desktop/UW_dataset');

%% Define (manually) the subjects and trials eligible for the quick test
subjects = [15];

trialsShoes = struct;
trialsFP    = struct;

trialsShoes.Subject_01 = [2,3,4,5,6,7,8,9,11,12];
trialsFP.Subject_01    = [2,3,4,5,6,7,8,9,11,12];

trialsShoes.Subject_02 = [1,2,3,4,5,6,7,8,9, 10];
trialsFP.Subject_02    = [1,2,3,4,5,7,8,9,11,12];

trialsShoes.Subject_03 = [1,2,3,4,6,8, 9, 10,11,12,13];
trialsFP.Subject_03    = [1,2,4,6,8,13,15,16,17,20,21];

trialsShoes.Subject_04 = [1,2,3,4,5,8, 9, 10,11,12,13];
trialsFP.Subject_04    = [1,2,5,6,7,16,18,19,20,22,23];

trialsShoes.Subject_05 = [1,2,3, 4, 5, 6, 10,9];
trialsFP.Subject_05    = [1,4,10,13,16,19,24,23];

trialsShoes.Subject_06 = [1,2,4,5,6, 7, 8, 9];
trialsFP.Subject_06    = [4,5,7,9,11,12,13,14];

trialsShoes.Subject_07 = [1,2,3,4,5,7, 8, 9, 10,11,13];
trialsFP.Subject_07    = [4,5,6,7,8,10,11,12,13,14,16];

trialsShoes.Subject_08 = [2,3,6,7,8,9,11,10,13];
trialsFP.Subject_08    = [2,3,5,6,7,8,9, 10,13];

trialsShoes.Subject_09 = [1,2,3,4,5,6,7,8,9,10,11];
trialsFP.Subject_09    = [1,2,3,4,5,6,7,8,9,10,11];

trialsShoes.Subject_10 = [1,2,3,4,5,6,7,8,10,11,12];
trialsFP.Subject_10    = [1,2,3,4,5,6,7,8,10,11,12];

trialsShoes.Subject_11 = [1,2,3,4,5,6,8,7,9,10,13];
trialsFP.Subject_11    = [1,2,3,4,5,6,8,7,9,10,13];

trialsShoes.Subject_12 = [1,2,3,4,5,6,7,8,9,10,11];
trialsFP.Subject_12    = [1,2,3,4,5,6,7,8,9,10,11];

trialsShoes.Subject_13 = [1,2,3,4,6,8,10,11,12,13,14];
trialsFP.Subject_13    = [1,2,3,4,7,9,11,12,14,15,16];

trialsShoes.Subject_14 = [1,2,3,4,5,6,7,8,9,10,11];
trialsFP.Subject_14    = [1,2,3,4,5,6,7,8,9,10,11];

trialsShoes.Subject_15 = [2,3,4,5,6,7,8,10,11,12,13];
trialsFP.Subject_15    = [3,4,5,6,7,8,9,11,12,13,14];

fields_shoes = fieldnames(trialsShoes);
fields_fp = fieldnames(trialsFP);

for subjectID = subjects
    valid_trialsShoes = trialsShoes.(fields_shoes{subjectID});
    valid_trialsFP    = trialsFP.(fields_fp{subjectID});
    
    % load forcplate file, 1 per subject
    forceplate_Offset= sprintf(fullfile(pathToFolder, ...
        'Subject_%02d/forceplates/unloaded_fp1.anc'), subjectID); 
    
    for trialID = 1 : size(valid_trialsShoes,2)  
    %% Load shoes    
    % LEFT---------------------------------------------------------------------  
    leftShoe_frontForce = sprintf(fullfile(pathToFolder, ...
            'Subject_%02d/shoes/dump_onlyDriverForces/ftShoeDriver_Left/frontForce_000%02d/data.log'),...
             subjectID, valid_trialsShoes(trialID));
    leftShoe_rearForce = sprintf(fullfile(pathToFolder, ...
            'Subject_%02d/shoes/dump_onlyDriverForces/ftShoeDriver_Left/rearForce_000%02d/data.log'),...
             subjectID, valid_trialsShoes(trialID));    
    leftShoe_totalForce = sprintf(fullfile(pathToFolder, ...
            'Subject_%02d/shoes/dump_onlyDriverForces/ftShoeDriver_Left/totalForce_000%02d/data.log'),...
             subjectID, valid_trialsShoes(trialID));

    % RIGHT--------------------------------------------------------------------
    rightShoe_frontForce = sprintf(fullfile(pathToFolder, ...
            'Subject_%02d/shoes/dump_onlyDriverForces/ftShoeDriver_Right/frontForce_000%02d/data.log'),...
             subjectID, valid_trialsShoes(trialID));
    rightShoe_rearForce = sprintf(fullfile(pathToFolder, ...
            'Subject_%02d/shoes/dump_onlyDriverForces/ftShoeDriver_Right/rearForce_000%02d/data.log'),...
             subjectID, valid_trialsShoes(trialID));    
    rightShoe_totalForce = sprintf(fullfile(pathToFolder, ...
            'Subject_%02d/shoes/dump_onlyDriverForces/ftShoeDriver_Right/totalForce_000%02d/data.log'),...
             subjectID, valid_trialsShoes(trialID));

    %% Parse shoes
    % LEFT---------------------------------------------------------------------
    shoes.Left.frontForce = parseYARPftShoes_fromDriver(leftShoe_frontForce);
    shoes.Left.rearForce  = parseYARPftShoes_fromDriver(leftShoe_rearForce);
    shoes.Left.totalForce = parseYARPftShoes_fromDriver(leftShoe_totalForce);

    % RIGHT--------------------------------------------------------------------
    shoes.Right.frontForce = parseYARPftShoes_fromDriver(rightShoe_frontForce);
    shoes.Right.rearForce  = parseYARPftShoes_fromDriver(rightShoe_rearForce);
    shoes.Right.totalForce = parseYARPftShoes_fromDriver(rightShoe_totalForce);
    
    %% Load forceplates
    forceplate_fileANC = sprintf(fullfile(pathToFolder, ...
        'Subject_%02d/forceplates/exercise%d.anc'), subjectID, valid_trialsFP(trialID)); 
 
    %% Parse forceplates
    % FP1 --> associated to LeftShoe
    % FP2 --> associated to RightShoe
    [FP1, FP2] = parseForceplates(forceplate_fileANC, forceplate_Offset);
    
    %% Plot raw data
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
                  box(axes1,'on');
                  hold(axes1,'on');
                  grid on;
    % --------------------------              
% % % % %     % FRONT             
% % % % %     subplot (211) 
% % % % %     plot1 = plot(shoes.Right.frontForce.forces(3,:),'b','lineWidth',1.5); % RIGHT
% % % % %     hold on
% % % % %     %meanFront
% % % % %     meanFront = zeros(size(shoes.Right.frontForce.forces(3,:)));
% % % % %     for i = 1: size(shoes.Right.frontForce.forces(3,:),2)
% % % % %         meanFront(i) = mean(shoes.Right.frontForce.forces(3,:));
% % % % %     end
% % % % %     plot2 = plot(meanFront,'b','lineWidth',1); % RIGHT
% % % % %     ylabel('Right','HorizontalAlignment','center',...
% % % % %            'FontWeight','bold',...
% % % % %            'FontSize',18);
% % % % %     title (['FRONT, Subject ' num2str(subjectID) ', Trial ' num2str(trialID)], 'FontSize',18);
% % % % %     grid on;
% % % % %     axis tight;
% % % % % 
% % % % %     subplot (212) 
% % % % %     plot1 = plot(shoes.Left.frontForce.forces(3,:),'r','lineWidth',1.5); % LEFT
% % % % %     hold on 
% % % % %     %meanFront
% % % % %     meanFront = zeros(size(shoes.Left.frontForce.forces(3,:)));
% % % % %     for i = 1: size(shoes.Left.frontForce.forces(3,:),2)
% % % % %         meanFront(i) = mean(shoes.Left.frontForce.forces(3,:));
% % % % %     end
% % % % %     plot2 = plot(meanFront,'r','lineWidth',1); % LEFT
% % % % %     ylabel('Left','HorizontalAlignment','center',...
% % % % %            'FontWeight','bold',...
% % % % %            'FontSize',18);
% % % % %     grid on;
% % % % %     axis tight;
% % % % %     ylabel('Left','HorizontalAlignment','center',...
% % % % %            'FontWeight','bold',...
% % % % %            'FontSize',18);
% % % % %     grid on;
% % % % %     axis tight;
% % % % %     % --------------------------
% % % % %     % REAR
% % % % %     figure
% % % % %     subplot (211) 
% % % % %     plot1 = plot(shoes.Right.rearForce.forces(3,:),'b','lineWidth',1.5); % RIGHT
% % % % %     hold on
% % % % %     %meanRear
% % % % %     meanRear = zeros(size(shoes.Right.rearForce.forces(3,:)));
% % % % %     for i = 1: size(shoes.Right.rearForce.forces(3,:),2)
% % % % %         meanRear(i) = mean(shoes.Right.rearForce.forces(3,:));
% % % % %     end
% % % % %     plot2 = plot(meanRear,'b','lineWidth',1); % RIGHT
% % % % %     ylabel('Right','HorizontalAlignment','center',...
% % % % %            'FontWeight','bold',...
% % % % %            'FontSize',18);
% % % % %     title (['REAR, Subject ' num2str(subjectID) ', Trial ' num2str(trialID)], 'FontSize',18);
% % % % %     grid on;
% % % % %     axis tight;
% % % % % 
% % % % %     subplot (212)
% % % % %     plot1 = plot(shoes.Left.rearForce.forces(3,:),'r','lineWidth',1.5); % LEFT
% % % % %     hold on
% % % % %     %meanRear
% % % % %     meanRear = zeros(size(shoes.Left.rearForce.forces(3,:)));
% % % % %     for i = 1: size(shoes.Left.rearForce.forces(3,:),2)
% % % % %         meanRear(i) = mean(shoes.Left.rearForce.forces(3,:));
% % % % % 
% % % % %     end
% % % % %     plot2 = plot(meanRear,'r','lineWidth',1); %% LEFT
% % % % %     ylabel('Left','HorizontalAlignment','center',...
% % % % %            'FontWeight','bold',...
% % % % %            'FontSize',18);
% % % % %     grid on;
% % % % %     axis tight;
    % -------------------------- 
    % TOTAL, SHOES
    subplot (321) 
    plot1 = plot(shoes.Right.totalForce.forces(3,:),'b','lineWidth',1.5); % RIGHT
    hold on
    %meanTotal
    meanTotal_right = zeros(size(shoes.Right.totalForce.forces(3,:)));
    for i = 1: size(shoes.Right.totalForce.forces(3,:),2)
        meanTotal_right(i) = mean(shoes.Right.totalForce.forces(3,:));
    end
    plot2 = plot(meanTotal_right,'b','lineWidth',1); % RIGHT
    ylabel('Right','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18);
    title (['SHOES, Subject ' num2str(subjectID) ', Trial ' num2str(valid_trialsShoes(trialID))], ...
          'FontSize',18);
    grid on;
    axis tight;

    subplot (323)
    plot1 = plot(shoes.Left.totalForce.forces(3,:),'r','lineWidth',1.5); % LEFT
    hold on
    %meanTotal
    meanTotal_left = zeros(size(shoes.Left.totalForce.forces(3,:)));
    weight_shoes = zeros(size(shoes.Left.totalForce.forces(3,:)));
    for i = 1: size(shoes.Left.totalForce.forces(3,:),2)
        meanTotal_left(i) = mean(shoes.Left.totalForce.forces(3,:));
        weight_shoes(i) = meanTotal_left(i)+meanTotal_right(1);
    end
    plot2 = plot(meanTotal_left,'r','lineWidth',1); % LEFT
    ylabel('Left','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18);
    grid on;
    axis tight;

    subplot (325) % weight from shoes
    plot1 = plot(weight_shoes,'black','lineWidth',1.5); 
    hold on 
    ylabel('w-shoes','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18);
    grid on;
    axis tight;
    % -------------------------- 
    % FORCEPLATES
    subplot (322) %FP2
    plot1 = plot(FP2.wrenches(:,3),'b','lineWidth',1.5); 
    hold on
    %meanFP2
    mean_FP2 = zeros(size(FP2.wrenches(:,3)));
    for i = 1: size(FP2.wrenches(:,3),1)
        mean_FP2(i) = mean(FP2.wrenches(:,3));
    end
    plot2 = plot(mean_FP2,'b','lineWidth',1); 
    ylabel('FP2','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18);
    title (['FP, Subject ' num2str(subjectID) ', Trial ' num2str(valid_trialsFP(trialID))], ...
           'FontSize',18);
    grid on;
    axis tight;
    
    subplot (324) %FP1
    plot1 = plot(FP1.wrenches(:,3),'r','lineWidth',1.5); 
    hold on
    %meanFP1
    mean_FP1 = zeros(size(FP1.wrenches(:,3)));
    weight_fp = zeros(size(FP1.wrenches(:,3)));
    for i = 1: size(FP1.wrenches(:,3),1)
        mean_FP1(i) = mean(FP1.wrenches(:,3));
        weight_fp(i) = mean_FP1(i)+mean_FP2(1);
    end
    plot2 = plot(mean_FP1,'r','lineWidth',1);
    ylabel('FP1','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18);
    grid on;
    axis tight;
    
    subplot (326) % weight from fp
    plot1 = plot(weight_fp,'black','lineWidth',1.5); 
    hold on 
    ylabel('w-fp','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18);
    grid on;
    axis tight;

    end
end
