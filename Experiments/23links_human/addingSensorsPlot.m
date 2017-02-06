% Create folder for plots
figFolder = fullfile(pwd,'/plots');
if(exist(figFolder,'dir')==0)
    mkdir(figFolder);
end

%% ------------------------------------------------------------------------
% ANKLE SX

fig1 = figure();
axes1 = axes('Parent',fig1,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;
title('LeftAnkle');

%MAP 2 sens
col = [0.87058824300766 0.490196079015732 0];
tauAnkle_sx_2sens = mu_dgiveny_2sens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_roty'),:);
shad1 = shadedErrorBar([],tauAnkle_sx_2sens,2.*sqrt(Sigma_specific_2sens(:,1)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

%MAP 3 sens
col = [0.494117647409439 0.184313729405403 0.556862771511078];
tauAnkle_sx_3sens = mu_dgiveny_3sens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_roty'),:);
shad2 = shadedErrorBar([],tauAnkle_sx_3sens,2.*sqrt(Sigma_specific_3sens(:,1)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

%MAP 4 sens
col = [0.466666668653488 0.674509823322296 0.18823529779911];
tauAnkle_sx_ALLsens = mu_dgiveny_ALLsens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_roty'),:);
shad3 = shadedErrorBar([],tauAnkle_sx_ALLsens,2.*sqrt(Sigma_specific_ALLsens(:,1)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

leg = legend([shad1.mainLine,shad1.patch,shad2.mainLine,shad2.patch,shad3.mainLine,shad3.patch],...
    {'$\mu_{\tau|{\ddot q, fp}}$','$2\sigma_{\tau|{\ddot q, fp}}$',...
    '$\mu_{\tau|{\ddot q, fp, robot}}$','$2\sigma_{\tau|{\ddot q, fp, robot}}$',...
    '$\mu_{\tau|{\ddot q, fp, robot, imu}}$','$2\sigma_{\tau|{\ddot q, fp, robot, imu}}$'},...
    'Location','northeast');
set(leg,'Interpreter','latex');
set(leg,'FontSize',18);

xlabel('Frames','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
ylabel('$\tau$ [Nm]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
axis tight;
box(axes1,'on');
grid on;

%% ------------------------------------------------------------------------
% ANKLE DX

fig1 = figure();
axes1 = axes('Parent',fig1,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;
title('RightAnkle');

%MAP 2 sens
col = [0.87058824300766 0.490196079015732 0];
tauAnkle_dx_2sens = mu_dgiveny_2sens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_roty'),:);
shad1 = shadedErrorBar([],tauAnkle_dx_2sens,2.*sqrt(Sigma_specific_2sens(:,3)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

%MAP 3 sens
col = [0.494117647409439 0.184313729405403 0.556862771511078];
tauAnkle_dx_3sens = mu_dgiveny_3sens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_roty'),:);
shad2 = shadedErrorBar([],tauAnkle_dx_3sens,2.*sqrt(Sigma_specific_3sens(:,3)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

%MAP 4 sens
col = [0.466666668653488 0.674509823322296 0.18823529779911];
tauAnkle_dx_ALLsens = mu_dgiveny_ALLsens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_roty'),:);
shad3 = shadedErrorBar([],tauAnkle_dx_ALLsens,2.*sqrt(Sigma_specific_ALLsens(:,3)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

leg = legend([shad1.mainLine,shad1.patch,shad2.mainLine,shad2.patch,shad3.mainLine,shad3.patch],...
    {'$\mu_{\tau|{\ddot q, fp}}$','$2\sigma_{\tau|{\ddot q, fp}}$',...
    '$\mu_{\tau|{\ddot q, fp, robot}}$','$2\sigma_{\tau|{\ddot q, fp, robot}}$',...
    '$\mu_{\tau|{\ddot q, fp, robot, imu}}$','$2\sigma_{\tau|{\ddot q, fp, robot, imu}}$'},...
    'Location','northeast');
set(leg,'Interpreter','latex');
set(leg,'FontSize',18);

xlabel('Frames','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
ylabel('$\tau$ [Nm]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
axis tight;
box(axes1,'on');
grid on;

%% ------------------------------------------------------------------------
% HIP SX

fig1 = figure();
axes1 = axes('Parent',fig1,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;
title('LeftHip');

%MAP 2 sens
col = [0.87058824300766 0.490196079015732 0];
tauHip_sx_2sens = mu_dgiveny_2sens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jLeftHip_roty'),:);
shad1 = shadedErrorBar([],tauHip_sx_2sens,2.*sqrt(Sigma_specific_2sens(:,2)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

%MAP 3 sens
col = [0.494117647409439 0.184313729405403 0.556862771511078];
tauHip_sx_3sens = mu_dgiveny_3sens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jLeftHip_roty'),:);
shad2 = shadedErrorBar([],tauHip_sx_3sens,2.*sqrt(Sigma_specific_3sens(:,2)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

%MAP 4 sens
col = [0.466666668653488 0.674509823322296 0.18823529779911];
tauHip_sx_ALLsens = mu_dgiveny_ALLsens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jLeftHip_roty'),:);
shad3 = shadedErrorBar([],tauHip_sx_ALLsens,2.*sqrt(Sigma_specific_ALLsens(:,2)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

leg = legend([shad1.mainLine,shad1.patch,shad2.mainLine,shad2.patch,shad3.mainLine,shad3.patch],...
    {'$\mu_{\tau|{\ddot q, fp}}$','$2\sigma_{\tau|{\ddot q, fp}}$',...
    '$\mu_{\tau|{\ddot q, fp, robot}}$','$2\sigma_{\tau|{\ddot q, fp, robot}}$',...
    '$\mu_{\tau|{\ddot q, fp, robot, imu}}$','$2\sigma_{\tau|{\ddot q, fp, robot, imu}}$'},...
    'Location','northeast');
set(leg,'Interpreter','latex');
set(leg,'FontSize',18);

xlabel('Frames','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
ylabel('$\tau$ [Nm]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
axis tight;
box(axes1,'on');
grid on;
%% ------------------------------------------------------------------------
% HIP DX

fig1 = figure();
axes1 = axes('Parent',fig1,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;
title('RightHip');

%MAP 2 sens
col = [0.87058824300766 0.490196079015732 0];
tauHip_dx_2sens = mu_dgiveny_2sens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jRightHip_roty'),:);
shad1 = shadedErrorBar([],tauHip_dx_2sens,2.*sqrt(Sigma_specific_2sens(:,4)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

%MAP 3 sens
col = [0.494117647409439 0.184313729405403 0.556862771511078];
tauHip_dx_3sens = mu_dgiveny_3sens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jRightHip_roty'),:);
shad2 = shadedErrorBar([],tauHip_dx_3sens,2.*sqrt(Sigma_specific_3sens(:,4)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

%MAP 4 sens
col = [0.466666668653488 0.674509823322296 0.18823529779911];
tauHip_dx_ALLsens = mu_dgiveny_ALLsens(rangeOfDynamicVariable( berdy, iDynTree.DOF_TORQUE, 'jRightHip_roty'),:);
shad3 = shadedErrorBar([],tauHip_dx_ALLsens,2.*sqrt(Sigma_specific_ALLsens(:,4)), ...
    {'LineWidth', 4.0, 'Color',  col},1); 

leg = legend([shad1.mainLine,shad1.patch,shad2.mainLine,shad2.patch,shad3.mainLine,shad3.patch],...
    {'$\mu_{\tau|{\ddot q, fp}}$','$2\sigma_{\tau|{\ddot q, fp}}$',...
    '$\mu_{\tau|{\ddot q, fp, robot}}$','$2\sigma_{\tau|{\ddot q, fp, robot}}$',...
    '$\mu_{\tau|{\ddot q, fp, robot, imu}}$','$2\sigma_{\tau|{\ddot q, fp, robot, imu}}$'},...
    'Location','northeast');
set(leg,'Interpreter','latex');
set(leg,'FontSize',18);

xlabel('Frames','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
ylabel('$\tau$ [Nm]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
axis tight;
box(axes1,'on');
grid on;

%% ------------------------------------------------------------------------
sample = 600; %generic sample (mid of the task)

fig1 = figure();
axes1 = axes('Parent',fig1,...
             'YGrid','on',...
             'XTickLabel',{'$\tau_{Ankle,L}$','$\tau_{Ankle,R}$','$\tau_{Hip,L}$','$\tau_{Hip,R}$'},...
             'XTick',[1.3 2.3 3.3 4.3],...
             'TickLabelInterpreter','latex',...
             'FontSize',18);
box(axes1,'off');
hold(axes1,'on');

Sd_tau_2sens = diag ([sqrt(Sigma_specific_2sens(sample,1)),sqrt(Sigma_specific_2sens(sample,3)),sqrt(Sigma_specific_2sens(sample,2)),sqrt(Sigma_specific_2sens(sample,4))]); % 4 arancioni
Sd_tau_3sens = diag ([sqrt(Sigma_specific_3sens(sample,1)),sqrt(Sigma_specific_3sens(sample,3)),sqrt(Sigma_specific_3sens(sample,2)),sqrt(Sigma_specific_3sens(sample,4))]); % 4 viola
Sd_tau_ALLsens = diag ([sqrt(Sigma_specific_ALLsens(sample,1)),sqrt(Sigma_specific_ALLsens(sample,3)),sqrt(Sigma_specific_ALLsens(sample,2)),sqrt(Sigma_specific_ALLsens(sample,4))]); % 4 verdi

Atau1 = diag(Sd_tau_2sens);
Atau2 = diag(Sd_tau_3sens);
Atau3 = diag(Sd_tau_ALLsens);

bar1 = bar([1, 2, 3, 4],[Atau1(1) Atau1(2) Atau1(3) Atau1(4)],0.3,'FaceColor',[0.87058824300766 0.490196079015732 0]); hold on;
bar2 = bar([1.3, 2.3, 3.3, 4.3],[Atau2(1) Atau2(2) Atau2(3) Atau2(4)],0.3,'FaceColor',[0.494117647409439 0.184313729405403 0.556862771511078]);
bar3 = bar([1.6, 2.6, 3.6, 4.6],[Atau3(1) Atau3(2) Atau3(3) Atau3(4)],0.3,'FaceColor',[0.466666668653488 0.674509823322296 0.18823529779911]);

leg = legend('$\ddot q, fp$','$\ddot q, fp, robot$','$\ddot q, fp, robot, imu$','Location','east');
set(leg,'Interpreter','latex');
set(leg,'FontSize',18);                
xlabel('joint torques','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',20,...
       'Interpreter','latex');
ylabel('standard deviation [Nm]','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',20,...
       'Interpreter','latex');
   
 save2pdf(fullfile(figFolder, ('covariancesBar_1')),fig1,600);

