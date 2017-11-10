
% -----------------------------------------------------------------------%
%  BUTTERFLY TASK --> ANALYSIS & PLOTS
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
%FIG1
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;
            
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
% % % acc plot legend
% % leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'});
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.369020817175207 0.95613614004149 0.303215550427647 0.0305007585806261], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

% % % put a title on the top of the subplots
% % ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% % text(0.5, 1,'\bf Linear Acceleration [m/s^2]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);
% % %in sensor frame

align_Ylabels(fig) % if there are 9 subplots, align the ylabels
% save
if saveON
    save2pdf(fullfile(figFolder, ('acc_butterfly_dx_comparison')),fig,600);
end

% FIG2
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;
           
% % -------------
% LEFT UPPER ARM
% acc.measured raw data (sensor frame)
acc.measured.leftUpperArm = data(9).meas(:,range_cut_plot);
acc.measured.leftUpperArm_sigma = data(12).var;

% simulated with y_simulated
range_accMEAS_leftUpperArm = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'LeftUpperArm_accelerometer');
acc.simulated.leftUpperArm = y_simulated((range_accMEAS_leftUpperArm:range_accMEAS_leftUpperArm+5),:);
acc.simulated.leftUpperArm_sigma = Sigmay((range_accMEAS_leftUpperArm:range_accMEAS_leftUpperArm+5),(range_accMEAS_leftUpperArm:range_accMEAS_leftUpperArm+5));

subplot (331) %  left UpperArm x component
plot1 = plot(acc.simulated.leftUpperArm(1,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.leftUpperArm_sigma(1);
shad2 = shadedErrorBar([],acc.measured.leftUpperArm(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel('leftUpperArm','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
% title ('x');
grid on;

subplot (332) %  left UpperArm y component
plot1 = plot(acc.simulated.leftUpperArm(2,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.leftUpperArm_sigma(2);
shad2 = shadedErrorBar([],acc.measured.leftUpperArm(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
% title ('y');
grid on;

subplot (333) %  left UpperArm z component
plot1 = plot(acc.simulated.leftUpperArm(3,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.leftUpperArm_sigma(3);
shad2 = shadedErrorBar([],acc.measured.leftUpperArm(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
% title ('z');
grid on;

% % -------------
% LEFT FORE ARM
% acc.measured raw data (sensor frame)
acc.measured.leftForeArm = data(10).meas(:,range_cut_plot);
acc.measured.leftForeArm_sigma = data(12).var;

% simulated with y_simulated
range_accMEAS_leftForeArm = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'LeftForeArm_accelerometer');
acc.simulated.leftForeArm = y_simulated((range_accMEAS_leftForeArm:range_accMEAS_leftForeArm+5),:);
acc.simulated.leftForeArm_sigma = Sigmay((range_accMEAS_leftForeArm:range_accMEAS_leftForeArm+5),(range_accMEAS_leftForeArm:range_accMEAS_leftForeArm+5));

subplot (334) %  left ForeArm x component
plot1 = plot(acc.simulated.leftForeArm(1,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.leftForeArm_sigma(1);
shad2 = shadedErrorBar([],acc.measured.leftForeArm(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel('leftForeArm','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (335) %  left ForeArm y component
plot1 = plot(acc.simulated.leftForeArm(2,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.leftForeArm_sigma(2);
shad2 = shadedErrorBar([],acc.measured.leftForeArm(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
grid on;

subplot (336) %  left ForeArm z component
plot1 = plot(acc.simulated.leftForeArm(3,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.leftForeArm_sigma(3);
shad2 = shadedErrorBar([],acc.measured.leftForeArm(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
grid on;

% % -------------
% LEFT HAND
% acc.measured raw data (sensor frame)
acc.measured.leftHand = data(11).meas(:,range_cut_plot);
acc.measured.leftHand_sigma = data(7).var;

% simulated with y_simulated
range_accMEAS_leftHand = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'LeftHand_accelerometer');
acc.simulated.leftHand = y_simulated((range_accMEAS_leftHand:range_accMEAS_leftHand+5),:);
acc.simulated.leftHand_sigma = Sigmay((range_accMEAS_leftHand:range_accMEAS_leftHand+5),(range_accMEAS_leftHand:range_accMEAS_leftHand+5));

subplot (337) %  leftHand x component
plot1 = plot(acc.simulated.leftHand(1,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.leftHand_sigma(1);
shad2 = shadedErrorBar([],acc.measured.leftHand(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel('leftHand','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (338) %  leftHand y component
plot1 = plot(acc.simulated.leftHand(2,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.leftHand_sigma(2);
shad2 = shadedErrorBar([],acc.measured.leftHand(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
grid on;

subplot (339) %  leftHand z component
plot1 = plot(acc.simulated.leftHand(3,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = acc.measured.leftHand_sigma(3);
shad2 = shadedErrorBar([],acc.measured.leftHand(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlim([0 len])
grid on;

% % -------------
% % acc plot legend
% leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'},'Location','northeast');
% set(leg,'Interpreter','latex', ...
%        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
%        'Orientation','horizontal');
% set(leg,'FontSize',13);

align_Ylabels(fig) % if there are 9 subplots, align the ylabels
% save
if saveON
    save2pdf(fullfile(figFolder, ('acc_butterfly_sx_comparison')),fig,600);
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  LEGEND AS A FIGURE VALID FOR ALL THE PLOTS
% -----------------------------------------------------------------------%
% acc plot legend
% leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'});
% set(leg,'Interpreter','latex', ...
%        'Position',[0.369020817175207 0.95613614004149 0.303215550427647 0.0305007585806261], ...
%        'Orientation','horizontal');
% set(leg,'FontSize',13);
% 
% plot(leg);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  JOINT ACCELERATION
% -----------------------------------------------------------------------%
%FIG1 
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;

% ddq.estimated (after MAP)
ddq.estimated.rightShoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightShoulder_rotx'),:);
ddq.estimated.rightShoulder.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightShoulder_roty'),:);
ddq.estimated.rightShoulder.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightShoulder_rotz'),:);
% ddq.measured (before MAP)
jointVar_range = (rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightShoulder_rotx'));
ddq.measured.rightShoulder.rotx = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightShoulder_rotx'),:);
ddq.measured.rightShoulder.roty = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightShoulder_roty'),:);
ddq.measured.rightShoulder.rotz = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightShoulder_rotz'),:);
ddq.measured.rightShoulder_sigma = Sigmay(jointVar_range,jointVar_range);
%---
% ddq.estimated (after MAP)
ddq.estimated.rightElbow.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightElbow_roty'),:);
ddq.estimated.rightElbow.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightElbow_rotz'),:);
% ddq.measured (before MAP)
ddq.measured.rightElbow.roty = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightElbow_roty'),:);
ddq.measured.rightElbow.rotz = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightElbow_rotz'),:);
%---
% ddq.estimated (after MAP)
ddq.estimated.rightWrist.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightWrist_rotx'),:);
ddq.estimated.rightWrist.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jRightWrist_rotz'),:);
% ddq.measured (before MAP)
ddq.measured.rightWrist.rotx = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightWrist_rotx'),:);
ddq.measured.rightWrist.rotz = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jRightWrist_rotz'),:);
%---

jointVar = ddq.measured.rightShoulder_sigma;

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
% title ('x');
grid on;

subplot (332) % right shoulder y component
plot1 = plot(ddq.estimated.rightShoulder.roty,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.rightShoulder.roty,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
% title ('y');
grid on;

subplot (333) % right shoulder z component
plot1 = plot(ddq.estimated.rightShoulder.rotz,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.rightShoulder.rotz,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
% title ('z');
grid on;

% % ----
subplot (334) % right elbow x component
ylabel(' rightElbow','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (335) % right elbow y component
plot1 = plot(ddq.estimated.rightElbow.roty,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.rightElbow.roty,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
grid on;

subplot (336) % right elbow z component
plot1 = plot(ddq.estimated.rightElbow.rotz,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.rightElbow.rotz,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
grid on;
% ----
subplot (337) % right wrist x component
plot1 = plot(ddq.estimated.rightWrist.rotx,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.rightWrist.rotx,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel(' rightWrist','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (338) % right wrist y component
xlim([0 len])
grid on;

subplot (339) % right wrist z component
plot1 = plot(ddq.estimated.rightWrist.rotz,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.rightWrist.rotz,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
grid on;

% % % ddq plot legend
% % leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'});
% % % legend('boxoff')
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

align_Ylabels(fig) % if there are 9 subplots, align the ylabels
% save
if saveON
    save2pdf(fullfile(figFolder, ('ddq_butterfly_dx_comparison')),fig,600);
end

%FIG2 
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;

% ddq.estimated (after MAP)
ddq.estimated.leftShoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jLeftShoulder_rotx'),:);
ddq.estimated.leftShoulder.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jLeftShoulder_roty'),:);
ddq.estimated.leftShoulder.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jLeftShoulder_rotz'),:);
% ddq.measured (before MAP)
jointVar_range = (rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jLeftShoulder_rotx'));
ddq.measured.leftShoulder.rotx = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jLeftShoulder_rotx'),:);
ddq.measured.leftShoulder.roty = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jLeftShoulder_roty'),:);
ddq.measured.leftShoulder.rotz = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jLeftShoulder_rotz'),:);
ddq.measured.leftShoulder_sigma = Sigmay(jointVar_range,jointVar_range);
%---
% ddq.estimated (after MAP)
ddq.estimated.leftElbow.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jLeftElbow_roty'),:);
ddq.estimated.leftElbow.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jLeftElbow_rotz'),:);
% ddq.measured (before MAP)
ddq.measured.leftElbow.roty = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jLeftElbow_roty'),:);
ddq.measured.leftElbow.rotz = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jLeftElbow_rotz'),:);
%---
% ddq.estimated (after MAP)
ddq.estimated.leftWrist.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jLeftWrist_rotx'),:);
ddq.estimated.leftWrist.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, 'jLeftWrist_rotz'),:);
% ddq.measured (before MAP)
ddq.measured.leftWrist.rotx = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jLeftWrist_rotx'),:);
ddq.measured.leftWrist.rotz = y(rangeOfSensorMeasurement(berdy, iDynTree.DOF_ACCELERATION_SENSOR, 'jLeftWrist_rotz'),:);
%---

jointVar = ddq.measured.leftShoulder_sigma;

subplot (331) % left shoulder x component
plot1 = plot(ddq.estimated.leftShoulder.rotx,'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = jointVar;
shad2 = shadedErrorBar([],ddq.measured.leftShoulder.rotx,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel('leftShoulder','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
% title ('x');
grid on;

subplot (332) % left shoulder y component
plot1 = plot(ddq.estimated.leftShoulder.roty,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.leftShoulder.roty,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
% title ('y');
grid on;

subplot (333) % left shoulder z component
plot1 = plot(ddq.estimated.leftShoulder.rotz,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.leftShoulder.rotz,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
% title ('z');
grid on;

% % ----
% % ----
subplot (334) % left elbow x component
ylabel(' rightElbow','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (335) % left elbow y component
plot1 = plot(ddq.estimated.leftElbow.roty,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.leftElbow.roty,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
grid on;

subplot (336) % left elbow z component
plot1 = plot(ddq.estimated.leftElbow.rotz,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.leftElbow.rotz,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
grid on;

% ----
subplot (337) % left wrist x component
plot1 = plot(ddq.estimated.leftWrist.rotx,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.leftWrist.rotx,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel(' leftWrist','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (338) % left wrist x component
xlim([0 len])
grid on;

subplot (339) % left wrist z component
plot1 = plot(ddq.estimated.leftWrist.rotz,'b','lineWidth',1.5);
hold on 
shad2 = shadedErrorBar([],ddq.measured.leftWrist.rotz,2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
grid on;

% % % ddq plot legend
% % leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'});
% % % legend('boxoff')
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

align_Ylabels(fig) % if there are 9 subplots, align the ylabels
% save
if saveON
    save2pdf(fullfile(figFolder, ('ddq_butterfly_sx_comparison')),fig,600);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  EXTERNAL FORCES
% -----------------------------------------------------------------------%
% only on Right Fooot because:
% - LeftFott is the base --> no possible
% - other applied (exernal) forces are null

% % fig = figure();
% % axes1 = axes('Parent',fig,'FontSize',16);
% %               box(axes1,'on');
% %               hold(axes1,'on');
% %               grid on;

% fext.measured in y vector (link frame)
range_fextMEAS_rightFoot = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'RightFoot');
fext.measured.rightFoot = y((range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5),:);
fext.measured.rightFoot_sigma = diag(Sigmay((range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5),(range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5)));

% fext.estimated (after MAP) 
range_fextEST_rightFoot = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'RightFoot');
fext.estimated.rightFoot = mu_dgiveny((range_fextEST_rightFoot:range_fextEST_rightFoot+5 ),:);

% subplot (131) % Right foot x component
% plot1 = plot(fext.estimated.rightFoot(1,:),'b','lineWidth',1.5);
% hold on 
% specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(1);
% shad2 = shadedErrorBar([],fext.measured.rightFoot(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
% ylabel(' rightFoot','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% title ('x');
% grid on;
% 
% subplot (132) % Right foot y component
% plot1 = plot(fext.estimated.rightFoot(2,:),'b','lineWidth',1.5);
% hold on 
% specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(2);
% shad2 = shadedErrorBar([],fext.measured.rightFoot(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
% xlim([0 len])
% title ('y');
% grid on;
% 
% subplot (133) % Right foot z component
% plot1 = plot(fext.estimated.rightFoot(3,:),'b','lineWidth',1.5);
% hold on 
% specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(3);
% shad2 = shadedErrorBar([],fext.measured.rightFoot(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
% xlim([0 len])
% title ('z');
% grid on;

% % % % -------------
% % % fext plot legend
% % leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

% % % put a title on the top of the subplots
% % ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% % text(0.5, 1,'\bf External force [N]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);
% % % in link frames
% % 

% % save
% if saveON
%     save2pdf(fullfile(figFolder, ('fext_butterfly_comparison')),fig,600);
% end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  JOINT TORQUE - VERSION WITHOUT PLOTTED SIGMA TAU
% -----------------------------------------------------------------------%
% tau.estimated (after MAP) right ankle
tau.estimated.rightAnkle.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_rotx'),:);
tau.estimated.rightAnkle.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_roty'),:);
tau.estimated.rightAnkle.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_rotz'),:);

% tau.estimated (after MAP) right knee
tau.estimated.rightKnee.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightKnee_roty'),:);
tau.estimated.rightKnee.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightKnee_rotz'),:);

% tau.estimated (after MAP) right hip
tau.estimated.rightHip.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_rotx'),:);
tau.estimated.rightHip.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_roty'),:);
tau.estimated.rightHip.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_rotz'),:);

% tau.estimated (after MAP) right C7shoulder
tau.estimated.rightC7Shoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightC7Shoulder_rotx'),:);

% tau.estimated (after MAP) right shoulder
tau.estimated.rightShoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_rotx'),:);
tau.estimated.rightShoulder.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_roty'),:);
tau.estimated.rightShoulder.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_rotz'),:);

% FIG1.1
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;
              
% FORCES PLOT FROM PREVIOUS SECTION
subplot (331) % Right foot x component
plot1 = plot(fext.estimated.rightFoot(1,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(1);
shad1 = shadedErrorBar([],fext.measured.rightFoot(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
ylabel(' rightFoot','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
% title ('x');
grid on;

subplot (332) % Right foot y component
plot1 = plot(fext.estimated.rightFoot(2,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(2);
shad2 = shadedErrorBar([],fext.measured.rightFoot(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
% title ('y');
grid on;

subplot (333) % Right foot z component
plot1 = plot(fext.estimated.rightFoot(3,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(3);
shad2 = shadedErrorBar([],fext.measured.rightFoot(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
xlim([0 len])
% title ('z');
grid on;

% ----
subplot (334) % right ankle x component
plot1 = plot(tau.estimated.rightAnkle.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.rotx,2.*sqrt(Sigma_tau.rightAnkle_rotx),'b',1.5); 
hold on 
ylabel('rightAnkle','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (335) % right ankle y component
plot1 = plot(tau.estimated.rightAnkle.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.roty,2.*sqrt(Sigma_tau.rightAnkle_roty),'b',1.5); 
xlim([0 len])
grid on;

subplot (336) % right ankle z component
plot1 = plot(tau.estimated.rightAnkle.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.rotz,2.*sqrt(Sigma_tau.rightAnkle_rotz),'b',1.5); 
xlim([0 len])
grid on;

% ----
subplot (337) % right knee x component
ylabel('rightKnee','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
xlim([0 len])
grid on;

subplot (338) % right knee y component
plot1 = plot(tau.estimated.rightKnee.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.rightKnee.roty,2.*sqrt(Sigma_tau.rightAnkle_roty),'b',1.5); 
xlim([0 len])
xlim([0 len])
grid on;

subplot (339) % right knee z component
plot1 = plot(tau.estimated.rightKnee.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightKnee.rotz,2.*sqrt(Sigma_tau.rightAnkle_rotz),'b',1.5); 
xlim([0 len])
grid on;
% % 
% % % tau plot legend
% % leg = legend([plot1,shad1.mainLine,shad1.patch],{'estim', 'meas','2$\sigma_{meas}$'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

align_Ylabels(fig) % if there are 9 subplots, align the ylabels
if saveON
    save2pdf(fullfile(figFolder, ('tau_bufferfly_dx_11')),fig,600);
end

% FIG1.2
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;

% ----
subplot (331) % right hip x component
plot1 = plot(tau.estimated.rightHip.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightHip.rotx,2.*sqrt(Sigma_tau.rightHip_rotx),'b',1.5); 
hold on 
ylabel('rightHip','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
% title ('x');
grid on;

subplot (332) % right hip y component
plot1 = plot(tau.estimated.rightHip.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.rightHip.roty,2.*sqrt(Sigma_tau.rightHip_roty),'b',1.5); 
xlim([0 len])
% title ('y');
grid on;

subplot (333) % right hip z component
plot1 = plot(tau.estimated.rightHip.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightHip.rotz,2.*sqrt(Sigma_tau.rightHip_rotz),'b',1.5); 
xlim([0 len])
% title ('z');
grid on;              
% ----
% ----
subplot (334) % right C7shoulder x component
plot1 = plot(tau.estimated.rightC7Shoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5); 
hold on 
ylabel('rightC7Shoulder','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (335) % right C7shoulder y component
xlim([0 len])
grid on;

subplot (336) % right C7shoulder z component
xlim([0 len])
grid on;

% ----
subplot (337) % right shoulder x component
plot1 = plot(tau.estimated.rightShoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotx,2.*sqrt(Sigma_tau.rightShoulder_rotx),'b',1.5); 
hold on 
ylabel('rightShoulder','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (338) % right shoulder y component
plot1 = plot(tau.estimated.rightShoulder.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.roty,2.*sqrt(Sigma_tau.rightShoulder_roty),'b',1.5); 
xlim([0 len])
grid on;

subplot (339) % right shoulder z component
plot1 = plot(tau.estimated.rightShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5); 
xlim([0 len])
grid on;
% % 
% % % tau plot legend
% % leg = legend([plot1],{'estim'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

align_Ylabels(fig) % if there are 9 subplots, align the ylabels
% save
if saveON
    save2pdf(fullfile(figFolder, ('tau_bufferfly_dx_12')),fig,600);
end

% -------------
% tau.estimated (after MAP) left ankle
tau.estimated.leftAnkle.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_rotx'),:);
tau.estimated.leftAnkle.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_roty'),:);
tau.estimated.leftAnkle.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_rotz'),:);

% tau.estimated (after MAP) left knee
tau.estimated.leftKnee.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftKnee_roty'),:);
tau.estimated.leftKnee.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftKnee_rotz'),:);

% tau.estimated (after MAP) left hip
tau.estimated.leftHip.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftHip_rotx'),:);
tau.estimated.leftHip.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftHip_roty'),:);
tau.estimated.leftHip.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftHip_rotz'),:);

% tau.estimated (after MAP) L5S1
tau.estimated.L5S1.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jL5S1_rotx'),:);
tau.estimated.L5S1.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jL5S1_roty'),:);

% tau.estimated (after MAP) left C7shoulder
tau.estimated.leftC7Shoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftC7Shoulder_rotx'),:);

% tau.estimated (after MAP) left shoulder
tau.estimated.leftShoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftShoulder_rotx'),:);
tau.estimated.leftShoulder.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftShoulder_roty'),:);
tau.estimated.leftShoulder.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftShoulder_rotz'),:);

% FIG2.1
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;
              
% L5S1
subplot (331) % L5S1 x component
plot1 = plot(tau.estimated.L5S1.rotx,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.L5S1.rotx,2.*sqrt(Sigma_tau.L5S1_rotx),'b',1.5); 
ylabel('L5S1','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
% title ('x');
grid on;

subplot (332) % L5S1 y component
plot1 = plot(tau.estimated.L5S1.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.L5S1.roty,2.*sqrt(Sigma_tau.L5S1_roty),'b',1.5); 
xlim([0 len])
% title ('y');
grid on;

subplot (333) % L5S1 z component
grid on;

% ----
subplot (334) % left ankle x component
plot1 = plot(tau.estimated.leftAnkle.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.rotx,2.*sqrt(Sigma_tau.leftAnkle_rotx),'b',1.5); 
hold on 
ylabel('leftAnkle','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (335) % left ankle y component
plot1 = plot(tau.estimated.leftAnkle.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.roty,2.*sqrt(Sigma_tau.leftAnkle_roty),'b',1.5); 
xlim([0 len])
grid on;

subplot (336) % left ankle z component
plot1 = plot(tau.estimated.leftAnkle.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.rotz,2.*sqrt(Sigma_tau.leftAnkle_rotz),'b',1.5); 
xlim([0 len])
grid on;

% ----
subplot (337) % left knee y component
% shad2 = shadedErrorBar([],tau.estimated.leftKnee.roty,2.*sqrt(Sigma_tau.leftAnkle_roty),'b',1.5); 
ylabel('leftKnee','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (338) % left knee y component
plot1 = plot(tau.estimated.leftKnee.roty,'b','lineWidth',1.5); 
xlim([0 len])
xlim([0 len])
grid on;

subplot (339) % left knee z component
plot1 = plot(tau.estimated.leftKnee.rotz,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.leftKnee.rotz,2.*sqrt(Sigma_tau.leftAnkle_rotz),'b',1.5); 
xlim([0 len])
grid on;

align_Ylabels(fig) % if there are 9 subplots, align the ylabels

% % % tau plot legend
% % leg = legend([plot1],{'estim'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

align_Ylabels(fig) % if there are 9 subplots, align the ylabels
if saveON
    save2pdf(fullfile(figFolder, ('tau_bufferfly_dx_21')),fig,600);
end

% FIG2.2
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;

% ----
subplot (331) % left hip x component
plot1 = plot(tau.estimated.leftHip.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftHip.rotx,2.*sqrt(Sigma_tau.leftHip_rotx),'b',1.5); 
hold on 
ylabel('leftHip','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
% title ('x');
grid on;

subplot (332) % left hip y component
plot1 = plot(tau.estimated.leftHip.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.leftHip.roty,2.*sqrt(Sigma_tau.leftHip_roty),'b',1.5); 
xlim([0 len])
% title ('y');
grid on;

subplot (333) % left hip z component
plot1 = plot(tau.estimated.leftHip.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftHip.rotz,2.*sqrt(Sigma_tau.leftHip_rotz),'b',1.5); 
xlim([0 len])
% title ('z');
grid on;

% ----
subplot (334) % left C7shoulder x component
plot1 = plot(tau.estimated.leftC7Shoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftC7Shoulder.rotx,2.*sqrt(Sigma_tau.leftC7Shoulder_rotx),'b',1.5); 
hold on 
ylabel('leftC7Shoulder','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (335) % left C7shoulder y component
xlim([0 len])
grid on;

subplot (336) % left C7shoulder z component
xlim([0 len])
grid on;

% ----
subplot (337) % left shoulder x component
plot1 = plot(tau.estimated.leftShoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftShoulder.rotx,2.*sqrt(Sigma_tau.leftShoulder_rotx),'b',1.5); 
hold on 
ylabel('leftShoulder','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0 len])
grid on;

subplot (338) % left shoulder y component
plot1 = plot(tau.estimated.leftShoulder.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.leftShoulder.roty,2.*sqrt(Sigma_tau.leftShoulder_roty),'b',1.5); 
xlim([0 len])
grid on;

subplot (339) % left shoulder z component
plot1 = plot(tau.estimated.leftShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftShoulder.rotz,2.*sqrt(Sigma_tau.leftShoulder_rotz),'b',1.5); 
xlim([0 len])
grid on;

% % % tau plot legend
% % leg = legend([plot1],{'estim'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

align_Ylabels(fig) % if there are 9 subplots, align the ylabels
% save
if saveON
    save2pdf(fullfile(figFolder, ('tau_bufferfly_dx_22')),fig,600);
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % -----------------------------------------------------------------------%
% %  JOINT TORQUE - VERSION WITH PLOTTED SIGMA TAU
% % -----------------------------------------------------------------------%
% % tau.estimated (after MAP) right ankle
% tau.estimated.rightAnkle.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_rotx'),:);
% tau.estimated.rightAnkle.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_roty'),:);
% tau.estimated.rightAnkle.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_rotz'),:);
% 
% % tau.estimated (after MAP) right knee
% tau.estimated.rightKnee.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightKnee_roty'),:);
% tau.estimated.rightKnee.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightKnee_rotz'),:);
% 
% % tau.estimated (after MAP) right hip
% tau.estimated.rightHip.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_rotx'),:);
% tau.estimated.rightHip.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_roty'),:);
% tau.estimated.rightHip.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_rotz'),:);
% 
% % tau.estimated (after MAP) right C7shoulder
% tau.estimated.rightC7Shoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightC7Shoulder_rotx'),:);
% 
% % tau.estimated (after MAP) right shoulder
% tau.estimated.rightShoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_rotx'),:);
% tau.estimated.rightShoulder.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_roty'),:);
% tau.estimated.rightShoulder.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_rotz'),:);
% 
% % FIG1.1
% fig = figure();
% axes1 = axes('Parent',fig,'FontSize',16);
%               box(axes1,'on');
%               hold(axes1,'on');
%               grid on;
%               
% % FORCES PLOT FROM PREVIOUS SECTION
% subplot (331) % Right foot x component
% plot1 = plot(fext.estimated.rightFoot(1,:),'b','lineWidth',1.5);
% hold on 
% specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(1);
% shad1 = shadedErrorBar([],fext.measured.rightFoot(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
% ylabel(' rightFoot','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% title ('x');
% grid on;
% 
% subplot (332) % Right foot y component
% plot1 = plot(fext.estimated.rightFoot(2,:),'b','lineWidth',1.5);
% hold on 
% specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(2);
% shad2 = shadedErrorBar([],fext.measured.rightFoot(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
% xlim([0 len])
% title ('y');
% grid on;
% 
% subplot (333) % Right foot z component
% plot1 = plot(fext.estimated.rightFoot(3,:),'b','lineWidth',1.5);
% hold on 
% specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(3);
% shad2 = shadedErrorBar([],fext.measured.rightFoot(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5); 
% xlim([0 len])
% title ('z');
% grid on;
% 
% % ----
% subplot (334) % right ankle x component
% % plot1 = plot(tau.estimated.rightAnkle.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.rotx,2.*sqrt(Sigma_tau.rightAnkle_rotx),'b',1.5); 
% hold on 
% ylabel('rightAnkle','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% grid on;
% 
% subplot (335) % right ankle y component
% % plot1 = plot(tau.estimated.rightAnkle.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.roty,2.*sqrt(Sigma_tau.rightAnkle_roty),'b',1.5); 
% xlim([0 len])
% grid on;
% 
% subplot (336) % right ankle z component
% % plot1 = plot(tau.estimated.rightAnkle.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.rotz,2.*sqrt(Sigma_tau.rightAnkle_rotz),'b',1.5); 
% xlim([0 len])
% grid on;
% 
% % ----
% subplot (338) % right knee y component
% % plot1 = plot(tau.estimated.rightAnkle.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.rightKnee.roty,2.*sqrt(Sigma_tau.rightAnkle_roty),'b',1.5); 
% ylabel('rightKnee','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% xlim([0 len])
% grid on;
% 
% subplot (339) % right knee z component
% % plot1 = plot(tau.estimated.rightAnkle.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightKnee.rotz,2.*sqrt(Sigma_tau.rightAnkle_rotz),'b',1.5); 
% xlim([0 len])
% grid on;
% 
% % tau plot legend
% leg = legend([plot1, shad2.mainLine, shad1.mainLine,shad1.patch],{'estim','2$\sigma_{estim}$', 'meas','2$\sigma_{meas}$'},'Location','northeast');
% set(leg,'Interpreter','latex', ...
%        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
%        'Orientation','horizontal');
% set(leg,'FontSize',13);
% 
% if saveON
%     save2pdf(fullfile(figFolder, ('tau_bufferfly_dx_11')),fig,600);
% end
% 
% % FIG1.2
% fig = figure();
% axes1 = axes('Parent',fig,'FontSize',16);
%               box(axes1,'on');
%               hold(axes1,'on');
%               grid on;
% 
% % ----
% subplot (331) % right hip x component
% % plot1 = plot(tau.estimated.rightHip.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightHip.rotx,2.*sqrt(Sigma_tau.rightHip_rotx),'b',1.5); 
% hold on 
% ylabel('rightHip','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% title ('x');
% grid on;
% 
% subplot (332) % right hip y component
% % plot1 = plot(tau.estimated.rightHip.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.rightHip.roty,2.*sqrt(Sigma_tau.rightHip_roty),'b',1.5); 
% xlim([0 len])
% title ('y');
% grid on;
% 
% subplot (333) % right hip z component
% % plot1 = plot(tau.estimated.rightHip.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightHip.rotz,2.*sqrt(Sigma_tau.rightHip_rotz),'b',1.5); 
% xlim([0 len])
% title ('z');
% grid on;              
% % ----
% % ----
% subplot (334) % right C7shoulder x component
% % plot1 = plot(tau.estimated.rightShoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5); 
% hold on 
% ylabel('rightC7Shoulder','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% grid on;
% 
% % ----
% subplot (337) % right shoulder x component
% % plot1 = plot(tau.estimated.rightShoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotx,2.*sqrt(Sigma_tau.rightShoulder_rotx),'b',1.5); 
% hold on 
% ylabel('rightShoulder','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% grid on;
% 
% subplot (338) % right shoulder y component
% % plot1 = plot(tau.estimated.rightShoulder.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.roty,2.*sqrt(Sigma_tau.rightShoulder_roty),'b',1.5); 
% xlim([0 len])
% grid on;
% 
% subplot (339) % right shoulder z component
% % plot1 = plot(tau.estimated.rightShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5); 
% xlim([0 len])
% grid on;
% 
% % % tau plot legend
% % leg = legend([plot1, shad2.mainLine, shad1.mainLine,shad1.patch],{'estim','2$\sigma_{estim}$', 'meas','2$\sigma_{meas}$'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);
% 
% % % % put a title on the top of the subplots
% % % ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% % % text(0.5, 1,'\bf Joint Torque [N/m]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);
% 
% % save
% if saveON
%     save2pdf(fullfile(figFolder, ('tau_bufferfly_dx_12')),fig,600);
% end
% 
% % -------------
% % tau.estimated (after MAP) left ankle
% tau.estimated.leftAnkle.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_rotx'),:);
% tau.estimated.leftAnkle.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_roty'),:);
% tau.estimated.leftAnkle.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_rotz'),:);
% 
% % tau.estimated (after MAP) left knee
% tau.estimated.leftKnee.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftKnee_roty'),:);
% tau.estimated.leftKnee.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftKnee_rotz'),:);
% 
% % tau.estimated (after MAP) left hip
% tau.estimated.leftHip.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftHip_rotx'),:);
% tau.estimated.leftHip.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftHip_roty'),:);
% tau.estimated.leftHip.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftHip_rotz'),:);
% 
% % tau.estimated (after MAP) L5S1
% tau.estimated.L5S1.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jL5S1_rotx'),:);
% tau.estimated.L5S1.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jL5S1_roty'),:);
% 
% % tau.estimated (after MAP) left C7shoulder
% tau.estimated.leftC7Shoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftC7Shoulder_rotx'),:);
% 
% % tau.estimated (after MAP) left shoulder
% tau.estimated.leftShoulder.rotx = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftShoulder_rotx'),:);
% tau.estimated.leftShoulder.roty = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftShoulder_roty'),:);
% tau.estimated.leftShoulder.rotz = mu_dgiveny(rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftShoulder_rotz'),:);
% 
% % FIG2.1
% fig = figure();
% axes1 = axes('Parent',fig,'FontSize',16);
%               box(axes1,'on');
%               hold(axes1,'on');
%               grid on;
%               
% % L5S1
% subplot (331) % L5S1 x component
% shad2 = shadedErrorBar([],tau.estimated.L5S1.rotx,2.*sqrt(Sigma_tau.L5S1_rotx),'b',1.5); 
% ylabel('L5S1','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% title ('x');
% grid on;
% 
% subplot (332) % L5S1 y component
% shad2 = shadedErrorBar([],tau.estimated.L5S1.roty,2.*sqrt(Sigma_tau.L5S1_roty),'b',1.5); 
% xlim([0 len])
% title ('y');
% grid on;
% 
% % ----
% subplot (334) % left ankle x component
% % plot1 = plot(tau.estimated.leftAnkle.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.rotx,2.*sqrt(Sigma_tau.leftAnkle_rotx),'b',1.5); 
% hold on 
% ylabel('leftAnkle','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% grid on;
% 
% subplot (335) % left ankle y component
% % plot1 = plot(tau.estimated.leftAnkle.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.roty,2.*sqrt(Sigma_tau.leftAnkle_roty),'b',1.5); 
% xlim([0 len])
% grid on;
% 
% subplot (336) % left ankle z component
% % plot1 = plot(tau.estimated.leftAnkle.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.rotz,2.*sqrt(Sigma_tau.leftAnkle_rotz),'b',1.5); 
% xlim([0 len])
% grid on;
% 
% % ----
% subplot (338) % left knee y component
% % plot1 = plot(tau.estimated.leftAnkle.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.leftKnee.roty,2.*sqrt(Sigma_tau.leftAnkle_roty),'b',1.5); 
% ylabel('leftKnee','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% xlim([0 len])
% grid on;
% 
% subplot (339) % left knee z component
% % plot1 = plot(tau.estimated.leftAnkle.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftKnee.rotz,2.*sqrt(Sigma_tau.leftAnkle_rotz),'b',1.5); 
% xlim([0 len])
% grid on;
% 
% % tau plot legend
% leg = legend([plot1, shad2.mainLine, shad1.mainLine,shad1.patch],{'estim','2$\sigma_{estim}$', 'meas','2$\sigma_{meas}$'},'Location','northeast');
% set(leg,'Interpreter','latex', ...
%        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
%        'Orientation','horizontal');
% set(leg,'FontSize',13);
% 
% if saveON
%     save2pdf(fullfile(figFolder, ('tau_bufferfly_dx_21')),fig,600);
% end
% 
% % FIG2.2
% fig = figure();
% axes1 = axes('Parent',fig,'FontSize',16);
%               box(axes1,'on');
%               hold(axes1,'on');
%               grid on;
% 
% % ----
% subplot (331) % left hip x component
% % plot1 = plot(tau.estimated.leftHip.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftHip.rotx,2.*sqrt(Sigma_tau.leftHip_rotx),'b',1.5); 
% hold on 
% ylabel('leftHip','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% title ('x');
% grid on;
% 
% subplot (332) % left hip y component
% % plot1 = plot(tau.estimated.leftHip.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.leftHip.roty,2.*sqrt(Sigma_tau.leftHip_roty),'b',1.5); 
% xlim([0 len])
% title ('y');
% grid on;
% 
% subplot (333) % left hip z component
% % plot1 = plot(tau.estimated.leftHip.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftHip.rotz,2.*sqrt(Sigma_tau.leftHip_rotz),'b',1.5); 
% xlim([0 len])
% title ('z');
% grid on;
% 
% % ----
% subplot (334) % left C7shoulder x component
% % plot1 = plot(tau.estimated.leftShoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftC7Shoulder.rotx,2.*sqrt(Sigma_tau.leftC7Shoulder_rotx),'b',1.5); 
% hold on 
% ylabel('leftC7Shoulder','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% grid on;
% 
% % ----
% subplot (337) % left shoulder x component
% % plot1 = plot(tau.estimated.leftShoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftShoulder.rotx,2.*sqrt(Sigma_tau.leftShoulder_rotx),'b',1.5); 
% hold on 
% ylabel('leftShoulder','HorizontalAlignment','center',...
%        'FontWeight','bold',...
%        'FontSize',18,...
%        'Interpreter','latex');
% xlim([0 len])
% grid on;
% 
% subplot (338) % left shoulder y component
% % plot1 = plot(tau.estimated.leftShoulder.roty,'b','lineWidth',1.5); 
% shad2 = shadedErrorBar([],tau.estimated.leftShoulder.roty,2.*sqrt(Sigma_tau.leftShoulder_roty),'b',1.5); 
% xlim([0 len])
% grid on;
% 
% subplot (339) % left shoulder z component
% % plot1 = plot(tau.estimated.leftShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftShoulder.rotz,2.*sqrt(Sigma_tau.leftShoulder_rotz),'b',1.5); 
% xlim([0 len])
% grid on;
% 
% % % tau plot legend
% % leg = legend([plot1, shad2.mainLine, shad1.mainLine,shad1.patch],{'estim','2$\sigma_{estim}$', 'meas','2$\sigma_{meas}$'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);
% 
% % % % put a title on the top of the subplots
% % % ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% % % text(0.5, 1,'\bf Joint Torque [N/m]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);
% 
% % save
% if saveON
%     save2pdf(fullfile(figFolder, ('tau_bufferfly_dx_22')),fig,600);
% end
% 
