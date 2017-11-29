
% -----------------------------------------------------------------------%
% PLOTS FOR VALIDATION SHOES an FP
% -----------------------------------------------------------------------%
%% WHY THIS ANALYSIS
% If we are able to validate the shoes with the forceplates, we can use 
% the shoes for filling the y vector of measurements

close all
len = length(comparison.forceplates.FP1.humanLeftFootWrench);
% len = 2500; only for squat task

figFolder = fullfile(bucket.pathToTrial,'/plots');
if(exist(figFolder,'dir')==0)
    mkdir(figFolder);
end

labelComponents = {'x','y','z'};

saveON =  true;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot raw comparison (no common ref. frames!)
% % plotting raw z-axis forces from fp and shoes. At this stage they are not 
% % in a common reference frame --> they have to be transformed into the
% % human frames for both feet.
% 
% fig = figure();
% axes1 = axes('Parent',fig,'FontSize',16);
%               box(axes1,'on');
%               hold(axes1,'on');
%               grid on;
% 
% subplot (211) % only z axis FP1-Left
% plot1 = plot(comparison.forceplates.downsampled.FP1.wrenches(:,3),'b','lineWidth',1.5);
% hold on 
% plot2 = plot(comparison.shoes.Left.forces(3,:),'r','lineWidth',1.5);
% ylabel('FP1-Left','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0  size(comparison.shoes.Left.forces(3,:),2)])
% grid on;
% 
% subplot (212) %only z axis FP2-Right
% plot1 = plot(comparison.forceplates.downsampled.FP2.wrenches(:,3),'b','lineWidth',1.5);
% hold on 
% plot2 = plot(comparison.shoes.Right.forces(3,:),'r','lineWidth',1.5);
% ylabel('FP2-Right','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0  size(comparison.shoes.Left.forces(3,:),2)])   
% grid on;
% 
% leg = legend([plot1,plot2],{'fp','shoe'});
% set(leg,'Interpreter','latex', ...
%        'Position',[0.369020817175207 0.95613614004149 0.303215550427647 0.0305007585806261], ...
%        'Orientation','horizontal');
% set(leg,'FontSize',13);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot comparison in human frames
% forces and moments expressed in human frames:
% - FP1-leftShoe expressed in leftFoot
% - FP2-rightShoes expressed in rightFoot

%% ----------FP1-LEFT SHOE COMPARISON
% -----FORCES
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
          box(axes1,'on');
          hold(axes1,'on');
          grid on;           
for i = 1:3 
    subplot (3,1,i)
    plot1 = plot(comparison.forceplates.FP1.humanLeftFootWrench(i,:),'b','lineWidth',1.5);
    hold on 
    plot2 = plot(comparison.shoes.Left.humanFootWrench(i,:)- staticOffset.FP1_LeftShoe(i),'r','lineWidth',1.5);
    ylabel(labelComponents{i},'HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18);
    xlim([0  len]);
    set(gca,'FontSize',15)
    grid on;
    if i == 1
        %title
        title ('Force [N]','HorizontalAlignment','center',...
            'FontWeight','bold',...
            'FontSize',20); %,...
        % 'Interpreter','latex');
        %legend
        leg = legend([plot1,plot2],{'fp1','LeftShoe'});
        set(leg,'Interpreter','latex');
        set(leg,'FontSize',20);
    end
    if i == 3
        xlabel('samples');
    end
    align_Ylabels(fig) % if there are  subplots, align the ylabels
end
if saveON
    save2pdf(fullfile(figFolder, ('fp1_leftShoe_FORCE_val')),fig,600);
end

% -----MOMENTS
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
          box(axes1,'on');
          hold(axes1,'on');
          grid on;
for i = 1:3 
    subplot (3,1,i)
    plot1 = plot(comparison.forceplates.FP1.humanLeftFootWrench(i+3,:),'b','lineWidth',1.5);
    hold on 
    plot2 = plot(comparison.shoes.Left.humanFootWrench(i+3,:) - staticOffset.FP1_LeftShoe(i+3),'r','lineWidth',1.5);
    ylabel(labelComponents{i},'HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18);
    xlim([0  len]);
    set(gca,'FontSize',15)
    grid on;
    if i == 1
        %title
        title ('Moment [Nm]','HorizontalAlignment','center',...
            'FontWeight','bold',...
            'FontSize',20); %,...
        % 'Interpreter','latex');
        %legend
        leg = legend([plot1,plot2],{'fp1','LeftShoe'});
        set(leg,'Interpreter','latex');
        set(leg,'FontSize',20);
    end
    if i == 3
        xlabel('samples');
    end
    align_Ylabels(fig) % if there are  subplots, align the ylabels
end
if saveON
    save2pdf(fullfile(figFolder, ('fp1_leftShoe_MOMENT_val')),fig,600);
end

%% ----------FP2-RIGHT SHOE COMPARISON
% -----FORCES
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
          box(axes1,'on');
          hold(axes1,'on');
          grid on;   
          
for i = 1:3 
    subplot (3,1,i)
    plot1 = plot(comparison.forceplates.FP2.humanRightFootWrench(i,:),'b','lineWidth',1.5);
    hold on 
    plot2 = plot(comparison.shoes.Right.humanFootWrench(i,:)- staticOffset.FP2_RightShoe(i),'r','lineWidth',1.5);
    ylabel(labelComponents{i},'HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18);
    xlim([0  len]);
    set(gca,'FontSize',15)
    grid on;
    if i == 1
        %title
        title ('Force [N]','HorizontalAlignment','center',...
            'FontWeight','bold',...
            'FontSize',20); %,...
        % 'Interpreter','latex');
        %legend
        leg = legend([plot1,plot2],{'fp2','RightShoe'});
        set(leg,'Interpreter','latex');
        set(leg,'FontSize',20);
    end
    if i == 3
        xlabel('samples');
    end
    align_Ylabels(fig) % if there are  subplots, align the ylabels
end
if saveON
    save2pdf(fullfile(figFolder, ('fp2_rightShoe_FORCE_val')),fig,600);
end

% -----MOMENTS
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
          box(axes1,'on');
          hold(axes1,'on');
          grid on;
for i = 1:3 
    subplot (3,1,i)
    plot1 = plot(comparison.forceplates.FP2.humanRightFootWrench(i+3,:),'b','lineWidth',1.5);
    hold on 
    plot2 = plot(comparison.shoes.Right.humanFootWrench(i+3,:) - staticOffset.FP2_RightShoe(i+3),'r','lineWidth',1.5);
    ylabel(labelComponents{i},'HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18);
    xlim([0  len]);
    set(gca,'FontSize',15)
    grid on;
    if i == 1
        %title
        title ('Moment [Nm]','HorizontalAlignment','center',...
            'FontWeight','bold',...
            'FontSize',20); %,...
        % 'Interpreter','latex');
        %legend
        leg = legend([plot1,plot2],{'fp2','RightShoe'});
        set(leg,'Interpreter','latex');
        set(leg,'FontSize',20);
    end
    if i == 3
        xlabel('samples');
    end
    align_Ylabels(fig) % if there are  subplots, align the ylabels
end
if saveON
    save2pdf(fullfile(figFolder, ('fp2_rightShoe_MOMENT_val')),fig,600);
end

%% RMSE computation
% RMSE = sqrt(mean(real-estim)^2) --> number value!
% hp: real --> fp, estim -->shoes
% since it could be present NaN values in one or both of real and estim,
% just removed first and last 5 samples from both signals.

comparison.RMSE.fp1 = comparison.forceplates.FP1.humanLeftFootWrench;
comparison.RMSE.leftShoe = comparison.shoes.Left.humanFootWrench - staticOffset.FP1_LeftShoe;
comparison.RMSE.fp1 = comparison.RMSE.fp1(:,5:end-5);
comparison.RMSE.leftShoe = comparison.RMSE.leftShoe(:,5:end-5);

comparison.RMSE.fp2 = comparison.forceplates.FP2.humanRightFootWrench;
comparison.RMSE.rightShoe = comparison.shoes.Right.humanFootWrench - staticOffset.FP2_RightShoe;
comparison.RMSE.fp2 = comparison.RMSE.fp2(:,5:end-5);
comparison.RMSE.rightShoe = comparison.RMSE.rightShoe(:,5:end-5);


comparison.RMSE.err1 = zeros(6,1);
comparison.RMSE.err2 = zeros(6,1);
for i =1 : 6
    comparison.RMSE.err1(i) = sqrt(mean((comparison.RMSE.fp1(i,:) - comparison.RMSE.leftShoe(i,:)).^2));
    comparison.RMSE.err2(i) = sqrt(mean((comparison.RMSE.fp2(i,:) - comparison.RMSE.rightShoe(i,:)).^2));
end

RMSE.fp1_leftShoe = comparison.RMSE.err1;
RMSE.fp2_rightShoe = comparison.RMSE.err2;
save(fullfile(bucket.pathToProcessedData,'/RMSE.mat'),'RMSE');
