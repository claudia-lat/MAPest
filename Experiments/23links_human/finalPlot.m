% FINAL PLOT

% Create folder for plots
figFolder = fullfile(pwd,'/plots');
if(exist(figFolder,'dir')==0)
    mkdir(figFolder);
end
% -------------------------------------------------------------------------
%% Plot 1
% Description here: torques on ankles and hips (along y axis) + related
% joint angles.
%%%%%%%%%%%%%
% LEFT
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

subplot (221)
plot1 = plot (mu_dgiveny(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_roty'),:),'lineWidth',1); hold on;
set(plot1,'color',[1 0 0]);
plot2 = plot (mu_dgiveny(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jLeftHip_roty'),:),'lineWidth',1); hold on;
set(plot2,'color',[0 0.498039215803146 0]);
ylabel('$\tau$ [Nm]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',13,...
       'Interpreter','latex');
ylim([-15 31])
xlim([0 1058])
grid on;  

subplot(223)
plot1 = plot(human_state.q(valueFromName(selectedJoints, 'jLeftAnkle_roty'),:)*180/pi,'lineWidth',1.5); hold on;
set(plot1,'color',[1 0 0]);
plot2 = plot(human_state.q(valueFromName(selectedJoints, 'jLeftHip_roty'),:)*180/pi,'lineWidth',1.5); hold on;
set(plot2,'color',[0 0.498039215803146 0]);
leg = legend('Left Ankle','Left Hip','Location','north');
set(leg,'Interpreter','latex');
set(leg,'FontSize',8)
xlabel('Frames','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',11,...
       'Interpreter','latex');
ylabel('$q$ [deg]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',13,...
       'Interpreter','latex');
ylim([-25 14])
xlim([0 1058])
grid on;  

%%%%%%%%%%%%%
% RIGHT

subplot (222)
plot1 = plot (mu_dgiveny(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_roty'),:),'lineWidth',1); hold on;
set(plot1,'color',[1 0 0]);
plot2 = plot (mu_dgiveny(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jRightHip_roty'),:),'lineWidth',1); hold on;
set(plot2,'color',[0 0.498039215803146 0]);
ylabel('$\tau$ [Nm]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',13,...
       'Interpreter','latex');
ylim([-15 31])
xlim([0 1058])
grid on; 
    
subplot(224)
plot1 = plot(human_state.q(valueFromName(selectedJoints, 'jRightAnkle_roty'),:)*180/pi,'lineWidth',1.5); hold on;
set(plot1,'color',[1 0 0]);
plot2 = plot(human_state.q(valueFromName(selectedJoints, 'jRightHip_roty'),:)*180/pi,'lineWidth',1.5); hold on;
set(plot2,'color',[0 0.498039215803146 0]);
leg = legend('Right Ankle','Right Hip','Location','north');
set(leg,'Interpreter','latex');
set(leg,'FontSize',8)
xlabel('Frames','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',11,...
       'Interpreter','latex');
ylabel('$q$ [deg]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',13,...
       'Interpreter','latex');
ylim([-25 14])
xlim([0 1058])
grid on;  

save2pdf(fullfile(figFolder, ('tau_LeftAndRight_fullSENS')),fig,600);
% -------------------------------------------------------------------------
%% Plot 2
% Description here: without Left Hand sensor, comparing external forces in
% y vector related to LeftHand and its estimation in d.
%%%%%%%%%%%%%
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

subplot (121)
plot1 = plot(y(415:417,:)','lineWidth',1.5);
leg = legend('$f^x_x$','$f^x_y$','$f^x_z$','Location','northeast');
set(leg,'Interpreter','latex');
set(leg,'FontSize',13);
ylabel('measured $f^x$ [N]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlabel('Frames','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',13,...
       'Interpreter','latex');
ylim([-50 90])
xlim([0 1058])
grid on;

subplot(122)
plot1 = plot (mu_dgiveny(1294:1296,:)','lineWidth',1.5);
leg = legend('$f^x_x$','$f^x_y$','$f^x_z$','Location','northeast');
set(leg,'Interpreter','latex');
set(leg,'FontSize',13);
ylabel('predicted $f^x$ [N]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlabel('Frames','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',13,...
       'Interpreter','latex');
ylim([-50 90])
xlim([0 1058])
grid on;  

save2pdf(fullfile(figFolder, ('fext_noLeftHand')),fig,600);
