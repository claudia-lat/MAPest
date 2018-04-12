% TEST FLOATING BASE
% comparing taus and fext

clear all;
close all;
clc;
%% Defining the torques vector for the comparison
bucket = struct;
subjectID = 10;
trialID = 5;
bucket.pathToSubject = sprintf(fullfile(pwd,'/dataUW/Subj_%02d'),subjectID);
bucket.pathToTrial   = sprintf(fullfile(bucket.pathToSubject,'/trial_0%02d'),trialID);
bucket.pathToProcessedData   = fullfile(bucket.pathToTrial,'/processed');

var_fromFloating = load(fullfile(bucket.pathToProcessedData,'/floating/estimated_variables.mat'),'estimated_variables');
var_fromFixed = load(fullfile(bucket.pathToProcessedData,'/shoes/estimated_variables.mat'),'estimated_variables');

range_cut_plot = (1:2000); %2000 is manually added in MAP computation

%% =========================== TAU COMPARISON =============================
% cutting rotx data
for i = 1 : length(var_fromFloating.estimated_variables.intForce.rotx)
    var_fromFloating.estimated_variables.intForce.rotx{i, 1}.projected_tau =  ...
        var_fromFloating.estimated_variables.intForce.rotx{i, 1}.projected_tau(:,range_cut_plot);
    var_fromFixed.estimated_variables.tau.rotx{i, 1}.tau =  ...
        var_fromFixed.estimated_variables.tau.rotx{i, 1}.tau(:,range_cut_plot);
end
% rotx plots             
for i = 1 : length(var_fromFloating.estimated_variables.intForce.rotx)
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;
    plot1 = plot(var_fromFloating.estimated_variables.intForce.rotx{i, 1}.projected_tau,'b','lineWidth',1.5);
    hold on;
    plot2 = plot(var_fromFixed.estimated_variables.tau.rotx{i, 1}.tau,'r','lineWidth',1.5);
    ylabel('Torque [Nm]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
   xlabel('N samples');
   title(sprintf('%s', var_fromFloating.estimated_variables.intForce.rotx{i, 1}.label), 'Interpreter', 'none');
   
   leg = legend([plot1,plot2],{'floating','fixed'});
   set(leg,'FontSize',13);
end

% -----------------------
% cutting roty data
for i = 1 : length(var_fromFloating.estimated_variables.intForce.roty)
    var_fromFloating.estimated_variables.intForce.roty{i, 1}.projected_tau =  ...
        var_fromFloating.estimated_variables.intForce.roty{i, 1}.projected_tau(:,range_cut_plot);
    var_fromFixed.estimated_variables.tau.roty{i, 1}.tau =  ...
        var_fromFixed.estimated_variables.tau.roty{i, 1}.tau(:,range_cut_plot);
end
% roty plots             
for i = 1 : length(var_fromFloating.estimated_variables.intForce.roty)
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;
    plot1 = plot(var_fromFloating.estimated_variables.intForce.roty{i, 1}.projected_tau,'b','lineWidth',1.5);
    hold on;
    plot2 = plot(var_fromFixed.estimated_variables.tau.roty{i, 1}.tau,'r','lineWidth',1.5);
    ylabel('Torque [Nm]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
   xlabel('N samples');
   title(sprintf('%s', var_fromFloating.estimated_variables.intForce.roty{i, 1}.label), 'Interpreter', 'none');
      
   leg = legend([plot1,plot2],{'floating','fixed'});
   set(leg,'FontSize',13);
end

% -----------------------
% cutting rotz data
for i = 1 : length(var_fromFloating.estimated_variables.intForce.rotz)
    var_fromFloating.estimated_variables.intForce.rotz{i, 1}.projected_tau =  ...
        var_fromFloating.estimated_variables.intForce.rotz{i, 1}.projected_tau(:,range_cut_plot);
    var_fromFixed.estimated_variables.tau.rotz{i, 1}.tau =  ...
        var_fromFixed.estimated_variables.tau.rotz{i, 1}.tau(:,range_cut_plot);
end
% roty plots             
for i = 1 : length(var_fromFloating.estimated_variables.intForce.rotz)
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;
    plot1 = plot(var_fromFloating.estimated_variables.intForce.rotz{i, 1}.projected_tau,'b','lineWidth',1.5);
    hold on;
    plot2 = plot(var_fromFixed.estimated_variables.tau.rotz{i, 1}.tau,'r','lineWidth',1.5);
    ylabel('Torque [Nm]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
   xlabel('N samples');
   title(sprintf('%s', var_fromFloating.estimated_variables.intForce.rotz{i, 1}.label), 'Interpreter', 'none');
   
   leg = legend([plot1,plot2],{'floating','fixed'});
   set(leg,'FontSize',13);
end

%% ======================= fext (fz) COMPARISON ===========================
baseLeftFoot = false;

if baseLeftFoot
    % cutting data
    for i = 1 : length(var_fromFixed.estimated_variables.extForce)
        var_fromFixed.estimated_variables.extForce{i, 1}.extForce = ...
            var_fromFixed.estimated_variables.extForce{i, 1}.extForce(3,range_cut_plot);
        var_fromFloating.estimated_variables.extForce{i+1, 1}.extForce = ...
            var_fromFloating.estimated_variables.extForce{i+1, 1}.extForce(3,range_cut_plot);
    end
    % text plots
    for i = 1 : length(var_fromFixed.estimated_variables.extForce)
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
                  box(axes1,'on');
                  hold(axes1,'on');
                  grid on;
        plot1 = plot(var_fromFloating.estimated_variables.extForce{i+1, 1}.extForce,'b','lineWidth',1.5);
        hold on;
        plot2 = plot(var_fromFixed.estimated_variables.extForce{i, 1}.extForce,'r','lineWidth',1.5);
        ylabel('Force [N]','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18,...
           'Interpreter','latex');
       xlabel('N samples');
       title(sprintf('%s', var_fromFixed.estimated_variables.extForce{i, 1}.label), 'Interpreter', 'none');

       leg = legend([plot1,plot2],{'floating','fixed'});
       set(leg,'FontSize',13);
    end

    % if the base is LeftFoot
    var_fromFloating.estimated_variables.extForce{1, 1}.extForce = ...
            var_fromFloating.estimated_variables.extForce{1, 1}.extForce(3,range_cut_plot);
    fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
                  box(axes1,'on');
                  hold(axes1,'on');
                  grid on;
    plot1 = plot(var_fromFloating.estimated_variables.extForce{1, 1}.extForce,'b','lineWidth',1.5);
    ylabel('Force [N]','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18,...
           'Interpreter','latex');
    xlabel('N samples');
    title('LeftFoot'); 
    leg = legend([plot1],{'floating'});
    set(leg,'FontSize',13);
    
else % i.e., RighFoot is the base --> tappullo!
    % I know that the RightFoot is the 19th in the list, so I manually
    % remove it!
    rightFoot_base = var_fromFloating.estimated_variables.extForce{19, 1};
    var_fromFloating.estimated_variables.extForce{19, 1} = [];
    var_fromFloating.estimated_variables.extForce = ...
        var_fromFloating.estimated_variables.extForce(~cellfun('isempty',var_fromFloating.estimated_variables.extForce));
    
    % cutting data
    rightFoot_base = rightFoot_base.extForce(3,range_cut_plot);
    for i = 1 : length(var_fromFixed.estimated_variables.extForce)
        var_fromFixed.estimated_variables.extForce{i, 1}.extForce = ...
            var_fromFixed.estimated_variables.extForce{i, 1}.extForce(3,range_cut_plot);
        var_fromFloating.estimated_variables.extForce{i, 1}.extForce = ...
            var_fromFloating.estimated_variables.extForce{i, 1}.extForce(3,range_cut_plot);
    end

   % text plots
    for i = 1 : length(var_fromFixed.estimated_variables.extForce)
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
                  box(axes1,'on');
                  hold(axes1,'on');
                  grid on;
        plot1 = plot(var_fromFloating.estimated_variables.extForce{i, 1}.extForce,'b','lineWidth',1.5);
        hold on;
        plot2 = plot(var_fromFixed.estimated_variables.extForce{i, 1}.extForce,'r','lineWidth',1.5);
        ylabel('Force [N]','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18,...
           'Interpreter','latex');
       xlabel('N samples');
       title(sprintf('%s', var_fromFixed.estimated_variables.extForce{i, 1}.label), 'Interpreter', 'none');

       leg = legend([plot1,plot2],{'floating','fixed'});
       set(leg,'FontSize',13);
    end
    
    % if the base is RightFoot
    fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
                  box(axes1,'on');
                  hold(axes1,'on');
                  grid on;
    plot1 = plot(rightFoot_base,'b','lineWidth',1.5);
    ylabel('Force [N]','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18,...
           'Interpreter','latex');
    xlabel('N samples');
    title('RightFoot'); 
    leg = legend([plot1],{'floating'});
    set(leg,'FontSize',13);
end