
% -----------------------------------------------------------------------%
%  PICK & PLACE TASK --> ANALYSIS & PLOTS
% -----------------------------------------------------------------------%
%% WHY THIS ANALYSIS
% If MAP is a able to estimate properly those variables that are also 
% measured (acc, fext and ddq) --> then we can suppose that the estimation
% of fint, fnet and tau are reasonable, as well.

close all
range_cut_plot = (1:2000); %2000 is manually added in MAP computation
human_state.q  = human_state.q(:,range_cut_plot);
human_state.dq = human_state.dq(:,range_cut_plot);
y = y(:,range_cut_plot);
mu_dgiveny = mu_dgiveny(:,range_cut_plot);

% simulate y vector from d
y_simulated = sim_y_test( berdy, human_state, mu_dgiveny);

len = size(mu_dgiveny,2);
specific_vector_sigma = zeros(1,len);

figFolder = fullfile(bucket.pathToTrial,'/plots');
if(exist(figFolder,'dir')==0)
    mkdir(figFolder);
end

saveON = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  ACCELERATION
% -----------------------------------------------------------------------%
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;
              
% % % RIGHT FOOT --> NO TOO MUCH NOISY
% % % acc.measured raw data (sensor frame)
% % acc.measured.rightFoot = data(14).meas(:,range_cut_plot);
% % acc.measured.rightFoot_sigma = data(14).var;
% % 
% % % simulated with y_simulated
% % range_accMEAS_rightFoot = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'RightFoot_accelerometer');
% % acc.simulated.rightFoot = y_simulated((range_accMEAS_rightFoot:range_accMEAS_rightFoot+5),:);
% % acc.simulated.rightFoot_sigma = Sigmay((range_accMEAS_rightFoot:range_accMEAS_rightFoot+5),(range_accMEAS_rightFoot:range_accMEAS_rightFoot+5));
% % 
% % 
% % subplot (331) % Right foot x component
% % plot1 = plot(acc.simulated.rightFoot(1,:),'b','lineWidth',1.5);
% % hold on 
% % specific_vector_sigma(1,:) = acc.measured.rightFoot_sigma(1);
% % shad2 = shadedErrorBar([],acc.measured.rightFoot(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
% % ylabel('rightFoot','HorizontalAlignment','center',...
% %        'FontWeight','bold',...
% %        'FontSize',18,...
% %        'Interpreter','latex');
% % xlim([0 len])
% % title ('x');
% % grid on;
% % 
% % subplot (332) %  Right foot y component
% % plot1 = plot(acc.simulated.rightFoot(2,:),'b','lineWidth',1.5);
% % hold on 
% % specific_vector_sigma(1,:) = acc.measured.rightFoot_sigma(2);
% % shad2 = shadedErrorBar([],acc.measured.rightFoot(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
% % xlim([0 len])
% % title ('y');
% % grid on;
% % 
% % subplot (333) %  Right foot z component
% % plot1 = plot(acc.simulated.rightFoot(3,:),'b','lineWidth',1.5);
% % hold on 
% % specific_vector_sigma(1,:) = acc.measured.rightFoot_sigma(3);
% % shad2 = shadedErrorBar([],acc.measured.rightFoot(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
% % xlim([0 len])
% % title ('z');
% % grid on;

% % -------------
% RIGHT UPPER ARM
% acc.measured raw data (sensor frame)
acc.measured.rightUpperArm = data(5).meas(:,range_cut_plot);
acc.measured.rightUpperArm_sigma = data(12).var;

% simulated with y_simulated
range_accMEAS_rightUpperArm = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'RightUpperArm_accelerometer');
acc.simulated.rightUpperArm = y_simulated((range_accMEAS_rightUpperArm:range_accMEAS_rightUpperArm+5),:);
acc.simulated.rightUpperArm_sigma = Sigmay((range_accMEAS_rightUpperArm:range_accMEAS_rightUpperArm+5),(range_accMEAS_rightUpperArm:range_accMEAS_rightUpperArm+5));

subplot (331) %  right UpperArm x component
plot1 = plot(acc.simulated.rightUpperArm(1,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.rightUpperArm_sigma(1);
shad2 = shadedErrorBar([],acc.measured.rightUpperArm(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel('rightUpperArm','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
title ('x');
grid on;

subplot (332) %  right UpperArm y component
plot1 = plot(acc.simulated.rightUpperArm(2,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.rightUpperArm_sigma(2);
shad2 = shadedErrorBar([],acc.measured.rightUpperArm(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
title ('y');
grid on;

subplot (333) %  right UpperArm z component
plot1 = plot(acc.simulated.rightUpperArm(3,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.rightUpperArm_sigma(3);
shad2 = shadedErrorBar([],acc.measured.rightUpperArm(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
title ('z');
grid on;

% % -------------
% RIGHT FORE ARM
% acc.measured raw data (sensor frame)
acc.measured.rightForeArm = data(6).meas(:,range_cut_plot);
acc.measured.rightForeArm_sigma = data(12).var;

% simulated with y_simulated
range_accMEAS_rightForeArm = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'RightForeArm_accelerometer');
acc.simulated.rightForeArm = y_simulated((range_accMEAS_rightForeArm:range_accMEAS_rightForeArm+5),:);
acc.simulated.rightForeArm_sigma = Sigmay((range_accMEAS_rightForeArm:range_accMEAS_rightForeArm+5),(range_accMEAS_rightForeArm:range_accMEAS_rightForeArm+5));

subplot (334) %  right ForeArm x component
plot1 = plot(acc.simulated.rightForeArm(1,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.rightForeArm_sigma(1);
shad2 = shadedErrorBar([],acc.measured.rightForeArm(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel('rightForeArm','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (335) %  right ForeArm y component
plot1 = plot(acc.simulated.rightForeArm(2,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.rightForeArm_sigma(2);
shad2 = shadedErrorBar([],acc.measured.rightForeArm(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
grid on;

subplot (336) %  right ForeArm z component
plot1 = plot(acc.simulated.rightForeArm(3,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.rightForeArm_sigma(3);
shad2 = shadedErrorBar([],acc.measured.rightForeArm(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
grid on;

% % -------------
% RIGHT HAND
% acc.measured raw data (sensor frame)
acc.measured.rightHand = data(7).meas(:,range_cut_plot);
acc.measured.rightHand_sigma = data(7).var;

% simulated with y_simulated
range_accMEAS_rightHand = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'RightHand_accelerometer');
acc.simulated.rightHand = y_simulated((range_accMEAS_rightHand:range_accMEAS_rightHand+5),:);
acc.simulated.rightHand_sigma = Sigmay((range_accMEAS_rightHand:range_accMEAS_rightHand+5),(range_accMEAS_rightHand:range_accMEAS_rightHand+5));

subplot (337) %  rightHand x component
plot1 = plot(acc.simulated.rightHand(1,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.rightHand_sigma(1);
shad2 = shadedErrorBar([],acc.measured.rightHand(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel('rightHand','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (338) %  rightHand y component
plot1 = plot(acc.simulated.rightHand(2,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.rightHand_sigma(2);
shad2 = shadedErrorBar([],acc.measured.rightHand(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
grid on;

subplot (339) %  rightHand z component
plot1 = plot(acc.simulated.rightHand(3,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.rightHand_sigma(3);
shad2 = shadedErrorBar([],acc.measured.rightHand(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
grid on;

% % -------------
% acc plot legend
leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'},'Location','northeast');
set(leg,'Interpreter','latex', ...
       'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
       'Orientation','horizontal');
set(leg,'FontSize',13);

% % % put a title on the top of the subplots
% % ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% % text(0.5, 1,'\bf Linear Acceleration [m/s^2]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);
% % %in sensor frame

% save
if saveON
    save2pdf(fullfile(figFolder, ('acc_p&p_comparison')),fig,600);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  EXTERNAL FORCES
% -----------------------------------------------------------------------%
% only on Right Fooot because:
% - LeftFott is the base --> no possible
% - other applied (exernal) forces are null
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;

% fext.measured in y vector (link frame)
range_fextMEAS_rightFoot = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'RightFoot');
fext.measured.rightFoot = y((range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5),:);
fext.measured.rightFoot_sigma = diag(Sigmay((range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5),(range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5)));

% fext.estimated (after MAP) 
range_fextEST_rightFoot = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'RightFoot');
fext.estimated.rightFoot = mu_dgiveny((range_fextEST_rightFoot:range_fextEST_rightFoot+5 ),:);

subplot (131) % Right foot x component
plot1 = plot(fext.estimated.rightFoot(1,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(1);
shad2 = shadedErrorBar([],fext.measured.rightFoot(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel(' rightFoot','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
title ('x');
grid on;

subplot (132) % Right foot y component
plot1 = plot(fext.estimated.rightFoot(2,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(2);
shad2 = shadedErrorBar([],fext.measured.rightFoot(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
title ('y');
grid on;

subplot (133) % Right foot z component
plot1 = plot(fext.estimated.rightFoot(3,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(3);
shad2 = shadedErrorBar([],fext.measured.rightFoot(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
title ('z');
grid on;

% % -------------
% fext plot legend
leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'},'Location','northeast');
set(leg,'Interpreter','latex', ...
       'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
       'Orientation','horizontal');
set(leg,'FontSize',13);

% % % put a title on the top of the subplots
% % ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% % text(0.5, 1,'\bf External force [N]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);
% % % in link frames
% % 

% save
if saveON
    save2pdf(fullfile(figFolder, ('fext_p&p_comparison')),fig,600);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  JOINT ACCELERATION
% -----------------------------------------------------------------------%
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;

% ddq.estimated (after MAP)
ddq.estimated.rightHip.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightHip_rotx'),:);
ddq.estimated.rightHip.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightHip_roty'),:);
ddq.estimated.rightHip.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightHip_rotz'),:);
% ddq.measured (before MAP)
jointVar_range = (rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightHip_rotx'));
ddq.measured.rightHip.rotx = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightHip_rotx'),:);
ddq.measured.rightHip.roty = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightHip_roty'),:);
ddq.measured.rightHip.rotz = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightHip_rotz'),:);
ddq.measured.rightHip_sigma = Sigmay(jointVar_range,jointVar_range);
%---
% ddq.estimated (after MAP)
ddq.estimated.leftHip.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jLeftHip_rotx'),:);
ddq.estimated.leftHip.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jLeftHip_roty'),:);
ddq.estimated.leftHip.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jLeftHip_rotz'),:);
% ddq.measured (before MAP)
ddq.measured.leftHip.rotx = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jLeftHip_rotx'),:);
ddq.measured.leftHip.roty = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jLeftHip_roty'),:);
ddq.measured.leftHip.rotz = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jLeftHip_rotz'),:);
%---
% ddq.estimated (after MAP)
ddq.estimated.rightShoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightShoulder_rotx'),:);
ddq.estimated.rightShoulder.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightShoulder_roty'),:);
ddq.estimated.rightShoulder.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightShoulder_rotz'),:);
% ddq.measured (before MAP)
ddq.measured.rightShoulder.rotx = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightShoulder_rotx'),:);
ddq.measured.rightShoulder.roty = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightShoulder_roty'),:);
ddq.measured.rightShoulder.rotz = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightShoulder_rotz'),:);

jointVar = ddq.measured.rightHip_sigma;

subplot (331) % right shoulder x component
plot1 = plot(ddq.estimated.rightShoulder.rotx,'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = jointVar;
shad2 = shadedErrorBar([],ddq.measured.rightShoulder.rotx,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel('rightShoulder','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
title ('x');
grid on;

subplot (332) % right shoulder y component
plot1 = plot(ddq.estimated.rightShoulder.roty,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.rightShoulder.roty,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
title ('y');
grid on;

subplot (333) % right shoulder z component
plot1 = plot(ddq.estimated.rightShoulder.rotz,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.rightShoulder.rotz,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
title ('z');
grid on;
% % ----
subplot (334) % right hip x component
plot1 = plot(ddq.estimated.rightHip.rotx,'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = jointVar;
shad2 = shadedErrorBar([],ddq.measured.rightHip.rotx,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel(' rightHip','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (335) % right hip y component
plot1 = plot(ddq.estimated.rightHip.roty,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.rightHip.roty,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
grid on;

subplot (336) % right hip z component
plot1 = plot(ddq.estimated.rightHip.rotz,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.rightHip.rotz,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
grid on;
% ----
subplot (337) % left hip x component
plot1 = plot(ddq.estimated.leftHip.rotx,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.leftHip.rotx,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel(' leftHip','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (338) % left hip y component
plot1 = plot(ddq.estimated.leftHip.roty,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.leftHip.roty,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
grid on;

subplot (339) % left hip z component
plot1 = plot(ddq.estimated.leftHip.rotz,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.leftHip.rotz,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
grid on;

% ddq plot legend
leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'});
% legend('boxoff')
set(leg,'Interpreter','latex', ...
       'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
       'Orientation','horizontal');
set(leg,'FontSize',13);

% % % put a title on the top of the subplots
% % ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% % text(0.5, 1,'\bf Joint Acceleration [rad/s^2]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

% save
if saveON
    save2pdf(fullfile(figFolder, ('ddq_p&p_comparison')),fig,600);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  JOINT TORQUE
% -----------------------------------------------------------------------%
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;

% tau.estimated (after MAP) right ankle
tau.estimated.rightAnkle.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_rotx'),:);
tau.estimated.rightAnkle.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_roty'),:);
tau.estimated.rightAnkle.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_rotz'),:);

% tau.estimated (after MAP) right hip
tau.estimated.rightHip.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_rotx'),:);
tau.estimated.rightHip.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_roty'),:);
tau.estimated.rightHip.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_rotz'),:);

% tau.estimated (after MAP) right shoulder
tau.estimated.rightShoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_rotx'),:);
tau.estimated.rightShoulder.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_roty'),:);
tau.estimated.rightShoulder.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_rotz'),:);

subplot (331) % right ankle x component
% plot1 = plot(tau.estimated.rightAnkle.rotx,'b','lineWidth',1.5);
shad2 = shadedErrorBar([],tau.estimated.rightAnkle.rotx,2.*sqrt(Sigma_tau.rightAnkle_rotx),'b',1.5); 
hold on 
ylabel('rightAnkle','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
title ('x');
grid on;

subplot (332) % right ankle y component
% plot1 = plot(tau.estimated.rightAnkle.roty,'b','lineWidth',1.5); 
shad2 = shadedErrorBar([],tau.estimated.rightAnkle.roty,2.*sqrt(Sigma_tau.rightAnkle_roty),'b',1.5); 
xlim([0 len])
title ('y');
grid on;

subplot (333) % right ankle z component
% plot1 = plot(tau.estimated.rightAnkle.rotz,'b','lineWidth',1.5);
shad2 = shadedErrorBar([],tau.estimated.rightAnkle.rotz,2.*sqrt(Sigma_tau.rightAnkle_rotz),'b',1.5); 
xlim([0 len])
title ('z');
grid on;

% ----
subplot (334) % right hip x component
% plot1 = plot(tau.estimated.rightHip.rotx,'b','lineWidth',1.5);
shad2 = shadedErrorBar([],tau.estimated.rightHip.rotx,2.*sqrt(Sigma_tau.rightHip_rotx),'b',1.5); 
hold on 
ylabel('rightHip','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (335) % right hip y component
% plot1 = plot(tau.estimated.rightHip.roty,'b','lineWidth',1.5); 
shad2 = shadedErrorBar([],tau.estimated.rightHip.roty,2.*sqrt(Sigma_tau.rightHip_roty),'b',1.5); 
xlim([0 len])
grid on;

subplot (336) % right hip z component
% plot1 = plot(tau.estimated.rightHip.rotz,'b','lineWidth',1.5);
shad2 = shadedErrorBar([],tau.estimated.rightHip.rotz,2.*sqrt(Sigma_tau.rightHip_rotz),'b',1.5); 
xlim([0 len])
grid on;

% ----
subplot (337) % right shoulder x component
% plot1 = plot(tau.estimated.rightShoulder.rotx,'b','lineWidth',1.5);
shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotx,2.*sqrt(Sigma_tau.rightShoulder_rotx),'b',1.5); 
hold on 
ylabel('rightShoulder','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (338) % right shoulder y component
% plot1 = plot(tau.estimated.rightShoulder.roty,'b','lineWidth',1.5); 
shad2 = shadedErrorBar([],tau.estimated.rightShoulder.roty,2.*sqrt(Sigma_tau.rightShoulder_roty),'b',1.5); 
xlim([0 len])
grid on;

subplot (339) % right shoulder z component
% plot1 = plot(tau.estimated.rightShoulder.rotz,'b','lineWidth',1.5);
shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5); 
xlim([0 len])
grid on;

% tau plot legend
leg = legend([shad2.mainLine,shad2.patch],{'$\tau$', '2$\sigma_{\tau}$'});
set(leg,'Interpreter','latex', ...
       'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
       'Orientation','horizontal');
set(leg,'FontSize',13);

% % % put a title on the top of the subplots
% % ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% % text(0.5, 1,'\bf Joint Torque [N/m]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

% save
if saveON
    save2pdf(fullfile(figFolder, ('tau_p&p_estimation')),fig,600);
end
