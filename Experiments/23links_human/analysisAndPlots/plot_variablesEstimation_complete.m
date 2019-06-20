
% -----------------------------------------------------------------------%
%  VARIABLES ESTIMATION --> ANALYSIS & PLOTS
% -----------------------------------------------------------------------%
close all;

%% WHY THIS ANALYSIS
% If MAP is able to estimate properly those variables that are also
% measured (acc, fext and ddq) --> then we can suppose that the estimation
% of fint, fnet and tau are reasonable, as well.

% variables in y simulated from d
load(fullfile(bucket.pathToProcessedData,'y_sim.mat'));

len = size(estimation.mu_dgiveny,2);
specific_vector_sigma = zeros(1,len);

% Plot folder
bucket.pathToPlots = fullfile(bucket.pathToTask,'plots');
if ~exist(bucket.pathToPlots)
    mkdir (bucket.pathToPlots)
end
saveON = true;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  ACCELERATION
% -----------------------------------------------------------------------%
% RIGHT
fig = figure('Name', 'Linear Acceleration - Right','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;
% % -------------
% RIGHT UPPER Leg
% simulated in y vector
range_accSIM_rightUpperLeg = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'RightUpperLeg_accelerometer');
acc.simulated.rightUpperLeg = y_sim((range_accSIM_rightUpperLeg:range_accSIM_rightUpperLeg+2),:);
% acc.simulated.rightUpperArm_sigma = Sigmay((range_accSIM_rightUpperArm:range_accSIM_rightUpperArm+2),(range_accSIM_rightUpperArm:range_accSIM_rightUpperArm+2));

% acc measured in y vector
acc.measured.rightUpperLeg = data(12).meas;
acc.measured.rightUpperLeg_sigma = data(12).var;

subplot (3,3,1) %  right x component
plot1 = plot(acc.simulated.rightUpperLeg(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.rightUpperLeg_sigma(1);
shad2 = shadedErrorBar([],acc.measured.rightUpperLeg(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
ylabel('rightUpperLeg','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
title ('x');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,2) %  right y component
plot1 = plot(acc.simulated.rightUpperLeg(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.rightUpperLeg_sigma(2);
shad2 = shadedErrorBar([],acc.measured.rightUpperLeg(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title ('y');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,3) %  right z component
plot1 = plot(acc.simulated.rightUpperLeg(3,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.rightUpperLeg_sigma(3);
shad2 = shadedErrorBar([],acc.measured.rightUpperLeg(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title ('z');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

% % -------------
% RIGHT lower Leg
% simulated in y vector
range_accSIM_rightLowerLeg = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'RightLowerLeg_accelerometer');
acc.simulated.rightLowerLeg = y_sim((range_accSIM_rightLowerLeg:range_accSIM_rightLowerLeg+2),:);
% acc.measured.rightForeArm_sigma = Sigmay((range_accSIM_rightForeArm:range_accSIM_rightForeArm+2),(range_accSIM_rightForeArm:range_accSIM_rightForeArm+2));

% acc measured in y vector
acc.measured.rightLowerLeg = data(13).meas;
acc.measured.rightLowerLeg_sigma = data(13).var;

subplot (3,3,4) %  right x component
plot1 = plot(acc.simulated.rightLowerLeg(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.rightLowerLeg_sigma(1);
shad2 = shadedErrorBar([],acc.measured.rightLowerLeg(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
ylabel('rightForeArm','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
grid on;
xlabel('N samples');
axis tight;
xlim([0 len])

subplot (3,3,5) %  right y component
plot1 = plot(acc.simulated.rightLowerLeg(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.rightLowerLeg_sigma(2);
shad2 = shadedErrorBar([],acc.measured.rightLowerLeg(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,6) %  right z component
plot1 = plot(acc.simulated.rightLowerLeg(3,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.rightLowerLeg_sigma(3);
shad2 = shadedErrorBar([],acc.measured.rightLowerLeg(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
xlabel('N samples');
axis tight;
xlim([0 len])

% % -------------
% % % % acc plot legend
% leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'});
% set(leg,'Interpreter','latex', ...
%        'Location','top', ...
%        'Orientation','horizontal');
% set(leg,'FontSize',13);

% % put a title on the top of the subplots
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'\bf Linear Acceleration [m/s^2]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('acc_dx_comparison')),fig,600);
end

%%
% LEFT
fig = figure('Name', 'Linear Acceleration - Right','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;
% % -------------
% LEFT UPPER ARM
% simulated in y vector
range_accSIM_leftUpperArm = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'LeftUpperArm_accelerometer');
acc.simulated.leftUpperArm = y_sim((range_accSIM_leftUpperArm:range_accSIM_leftUpperArm+2),:);
% acc.measured.leftUpperArm_sigma = Sigmay((range_accSIM_leftUpperArm:range_accSIM_leftUpperArm+2),(range_accSIM_leftUpperArm:range_accSIM_leftUpperArm+2));

% acc measured in y vector
acc.measured.leftUpperArm = data(9).meas;
acc.measured.leftUpperArm_sigma = data(9).var;

subplot (3,3,1) %  left x component
plot1 = plot(acc.simulated.leftUpperArm(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.leftUpperArm_sigma(1);
shad2 = shadedErrorBar([],acc.measured.leftUpperArm(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
ylabel('leftUpperArm','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
title ('x');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,2) %  left y component
plot1 = plot(acc.simulated.leftUpperArm(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.leftUpperArm_sigma(2);
shad2 = shadedErrorBar([],acc.measured.leftUpperArm(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title ('y');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,3) %  left z component
plot1 = plot(acc.simulated.leftUpperArm(3,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.leftUpperArm_sigma(3);
shad2 = shadedErrorBar([],acc.measured.leftUpperArm(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title ('z');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

% % -------------
% LEFT FORE ARM
% simulated in y vector
range_accSIM_leftForeArm = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'LeftForeArm_accelerometer');
acc.simulated.leftForeArm = y_sim((range_accSIM_leftForeArm:range_accSIM_leftForeArm+2),:);
% acc.measured.leftForeArm_sigma = Sigmay((range_accSIM_leftForeArm:range_accSIM_leftForeArm+2),(range_accSIM_leftForeArm:range_accSIM_leftForeArm+2));

% acc measured in y vector
acc.measured.leftForeArm = data(10).meas;
acc.measured.leftForeArm_sigma = data(10).var;

subplot (3,3,4) %  left x component
plot1 = plot(acc.simulated.leftForeArm(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.leftForeArm_sigma(1);
shad2 = shadedErrorBar([],acc.measured.rightForeArm(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
ylabel('rightForeArm','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
grid on;
xlabel('N samples');
axis tight;
xlim([0 len])

subplot (3,3,5) %  right y component
plot1 = plot(acc.simulated.leftForeArm(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.leftForeArm_sigma(2);
shad2 = shadedErrorBar([],acc.measured.leftForeArm(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,6) %  left z component
plot1 = plot(acc.simulated.leftForeArm(3,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = acc.measured.leftForeArm_sigma(3);
shad2 = shadedErrorBar([],acc.measured.leftForeArm(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
xlabel('N samples');
axis tight;
xlim([0 len])

% % put a title on the top of the subplots
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('acc_sx_comparison')),fig,600);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  EXTERNAL FORCES
% -----------------------------------------------------------------------%
% - other applied (exernal) forces are null

fig = figure('Name', 'External forces','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

% simulated in y vector
range_fextMEAS_rightFoot = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'RightFoot');
fext.measured.rightFoot = y_sim((range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5),:);
fext.measured.rightFoot_sigma = diag(Sigmay((range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5),(range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5)));

% % acc measured in y vector
range_fextEST_rightFoot = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'RightFoot');
fext.estimated.rightFoot = estimation.mu_dgiveny((range_fextEST_rightFoot:range_fextEST_rightFoot+5 ),:);

subplot (331) % Right foot x component
plot1 = plot(fext.estimated.rightFoot(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(1);
shad1 = shadedErrorBar([],fext.measured.rightFoot(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
ylabel(' rightFoot','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
title ('x');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (332) % Right foot y component
plot1 = plot(fext.estimated.rightFoot(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(2);
shad2 = shadedErrorBar([],fext.measured.rightFoot(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title ('y');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (333) % Right foot z component
plot1 = plot(fext.estimated.rightFoot(3,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(3);
shad2 = shadedErrorBar([],fext.measured.rightFoot(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title ('z');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

% simulated in y vector
range_fextMEAS_leftFoot = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'LeftFoot');
fext.measured.leftFoot = y_sim((range_fextMEAS_leftFoot:range_fextMEAS_leftFoot+5),:);
fext.measured.leftFoot_sigma = diag(Sigmay((range_fextMEAS_leftFoot:range_fextMEAS_leftFoot+5),(range_fextMEAS_leftFoot:range_fextMEAS_leftFoot+5)));

% acc measured in y vector
range_fextEST_leftFoot = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'LeftFoot');
fext.estimated.leftFoot = estimation.mu_dgiveny((range_fextEST_leftFoot:range_fextEST_leftFoot+5 ),:);

subplot (334) % left foot x component
plot1 = plot(fext.estimated.leftFoot(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftFoot_sigma(1);
shad1 = shadedErrorBar([],fext.measured.leftFoot(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
ylabel(' leftFoot','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
title ('x');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (335) % left foot y component
plot1 = plot(fext.estimated.leftFoot(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftFoot_sigma(2);
shad2 = shadedErrorBar([],fext.measured.leftFoot(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title ('y');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (336) % left foot z component
plot1 = plot(fext.estimated.leftFoot(3,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftFoot_sigma(3);
shad2 = shadedErrorBar([],fext.measured.leftFoot(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title ('z');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

% % % % -------------
% % % fext plot legend
% % leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

% % put a title on the top of the subplots
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'\bf External force [N]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('fext_comparison')),fig,600);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  JOINT TORQUE - VERSION WITHOUT PLOTTED SIGMA TAU
% -----------------------------------------------------------------------%
for tauIdx = 1 : nrDofs
    % (after MAP) right ankle
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightAnkle_rotx')
        tau.estimated.rightAnkle.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightAnkle_roty')
        tau.estimated.rightAnkle.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightAnkle_rotz')
        tau.estimated.rightAnkle.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) right knee
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightKnee_rotx')
        tau.estimated.rightKnee.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightKnee_roty')
        tau.estimated.rightKnee.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightKnee_rotz')
        tau.estimated.rightKnee.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) right hip
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightHip_rotx')
        tau.estimated.rightHip.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightHip_roty')
        tau.estimated.rightHip.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightHip_rotz')
        tau.estimated.rightHip.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) right C7shoulder
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightC7Shoulder_rotx')
        tau.estimated.rightC7Shoulder.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightC7Shoulder_roty')
        tau.estimated.rightC7Shoulder.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightC7Shoulder_rotz')
        tau.estimated.rightC7Shoulder.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) right shoulder
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightShoulder_rotx')
        tau.estimated.rightShoulder.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightShoulder_roty')
        tau.estimated.rightShoulder.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightShoulder_rotz')
        tau.estimated.rightShoulder.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) right elbow
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightElbow_rotx')
        tau.estimated.rightElbow.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightElbow_roty')
        tau.estimated.rightElbow.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightElbow_rotz')
        tau.estimated.rightElbow.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) right wrist
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightWrist_rotx')
        tau.estimated.rightWrist.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightWrist_roty')
        tau.estimated.rightWrist.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightWrist_rotz')
        tau.estimated.rightWrist.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % ---------------------
    % (after MAP) left ankle
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftAnkle_rotx')
        tau.estimated.leftAnkle.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftAnkle_roty')
        tau.estimated.leftAnkle.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftAnkle_rotz')
        tau.estimated.leftAnkle.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) left knee
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftKnee_rotx')
        tau.estimated.leftKnee.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftKnee_roty')
        tau.estimated.leftKnee.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftKnee_rotz')
        tau.estimated.leftKnee.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) left hip
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftHip_rotx')
        tau.estimated.leftHip.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftHip_roty')
        tau.estimated.leftHip.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftHip_rotz')
        tau.estimated.leftHip.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) left C7shoulder
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftC7Shoulder_rotx')
        tau.estimated.leftC7Shoulder.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLefttC7Shoulder_roty')
        tau.estimated.leftC7Shoulder.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftC7Shoulder_rotz')
        tau.estimated.leftC7Shoulder.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) left shoulder
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftShoulder_rotx')
        tau.estimated.leftShoulder.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftShoulder_roty')
        tau.estimated.leftShoulder.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftShoulder_rotz')
        tau.estimated.leftShoulder.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) left elbow
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftElbow_rotx')
        tau.estimated.leftElbow.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftElbow_roty')
        tau.estimated.leftElbow.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftElbow_rotz')
        tau.estimated.leftElbow.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
    % (after MAP) left wrist
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftWrist_rotx')
        tau.estimated.leftWrist.rotx = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftWrist_roty')
        tau.estimated.leftWrist.roty = estimatedVariables.tau.values(tauIdx,:);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftWrist_rotz')
        tau.estimated.leftWrist.rotz = estimatedVariables.tau.values(tauIdx,:);
    end
end

%% RIGHT ANKLE-KNEE-HIP
fig = figure('Name', 'Right leg torques','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;
% ----
subplot (331) % right ankle x component
plot1 = plot(tau.estimated.rightAnkle.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.rotx,2.*sqrt(Sigma_tau.rightAnkle_rotx),'b',1.5);
hold on
ylabel('rightAnkle','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
title ('x');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (332) % right ankle y component
plot1 = plot(tau.estimated.rightAnkle.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.roty,2.*sqrt(Sigma_tau.rightAnkle_roty),'b',1.5);
title ('y');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (333) % right ankle z component
plot1 = plot(tau.estimated.rightAnkle.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.rotz,2.*sqrt(Sigma_tau.rightAnkle_rotz),'b',1.5);
title ('z');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

% ----
subplot (334) % right knee x component
plot1 = plot(tau.estimated.rightKnee.rotx,'b','lineWidth',1.5);
ylabel('rightKnee','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (335) % right knee y component
plot1 = plot(tau.estimated.rightKnee.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightKnee.roty,2.*sqrt(Sigma_tau.rightAnkle_roty),'b',1.5);
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (336) % right knee z component
plot1 = plot(tau.estimated.rightKnee.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightKnee.rotz,2.*sqrt(Sigma_tau.rightAnkle_rotz),'b',1.5);
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (337) % right hip x component
plot1 = plot(tau.estimated.rightHip.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightHip.rotx,2.*sqrt(Sigma_tau.rightHip_rotx),'b',1.5);
hold on
ylabel('rightHip','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
% title ('x');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (338) % right hip y component
plot1 = plot(tau.estimated.rightHip.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightHip.roty,2.*sqrt(Sigma_tau.rightHip_roty),'b',1.5);
% title ('y');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (339) % right hip z component
plot1 = plot(tau.estimated.rightHip.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightHip.rotz,2.*sqrt(Sigma_tau.rightHip_rotz),'b',1.5);
% title ('z');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

% % % tau plot legend
% % leg = legend([plot1,shad1.mainLine,shad1.patch],{'estim', 'meas','2$\sigma_{meas}$'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

% % put a title on the top of the subplots
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'\bf Joint torque [Nm]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('tau_dx_leg')),fig,600);
end

%% LEFT ANKLE-KNEE-HIP
fig = figure('Name', 'Left leg torques','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

subplot (331) % left ankle x component
plot1 = plot(tau.estimated.leftAnkle.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.rotx,2.*sqrt(Sigma_tau.leftAnkle_rotx),'b',1.5);
hold on
ylabel('leftAnkle','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
title ('x');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (332) % left ankle y component
plot1 = plot(tau.estimated.leftAnkle.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.roty,2.*sqrt(Sigma_tau.leftAnkle_roty),'b',1.5);
title ('y');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (333) % left ankle z component
plot1 = plot(tau.estimated.leftAnkle.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.rotz,2.*sqrt(Sigma_tau.leftAnkle_rotz),'b',1.5);
title ('z');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (334) % left knee y component
plot1 = plot(tau.estimated.leftKnee.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftKnee.roty,2.*sqrt(Sigma_tau.leftAnkle_roty),'b',1.5);
ylabel('leftKnee','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (335) % left knee y component
plot1 = plot(tau.estimated.leftKnee.roty,'b','lineWidth',1.5);
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (336) % left knee z component
plot1 = plot(tau.estimated.leftKnee.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftKnee.rotz,2.*sqrt(Sigma_tau.leftAnkle_rotz),'b',1.5);
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

% ----
subplot (337) % left hip x component
plot1 = plot(tau.estimated.leftHip.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftHip.rotx,2.*sqrt(Sigma_tau.leftHip_rotx),'b',1.5);
hold on
ylabel('leftHip','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (338) % left hip y component
plot1 = plot(tau.estimated.leftHip.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftHip.roty,2.*sqrt(Sigma_tau.leftHip_roty),'b',1.5);
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (339) % left hip z component
plot1 = plot(tau.estimated.leftHip.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftHip.rotz,2.*sqrt(Sigma_tau.leftHip_rotz),'b',1.5);
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

% align_Ylabels(fig) % if there are 9 subplots, align the ylabels

% % % tau plot legend
% % leg = legend([plot1],{'estim'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

% % put a title on the top of the subplots
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'\bf Joint torque [Nm]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

tightfig();
% save
if saveON
    save2pdf(fullfile(figFolder, ('tau_sx_leg')),fig,600);
end

%% RIGHT ARM
fig = figure('Name', 'Right arm torques','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

subplot (431) % right shoulder x component
plot1 = plot(tau.estimated.rightShoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotx,2.*sqrt(Sigma_tau.rightShoulder_rotx),'b',1.5);
hold on
ylabel('rightShoulder','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('N samples');
title ('x');
grid on;
axis tight;
xlim([0 len])

subplot (432) % right shoulder y component
plot1 = plot(tau.estimated.rightShoulder.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.roty,2.*sqrt(Sigma_tau.rightShoulder_roty),'b',1.5);
xlabel('N samples');
title ('y');
grid on;
axis tight;
xlim([0 len])

subplot (433) % right shoulder z component
plot1 = plot(tau.estimated.rightShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('z');
grid on;
axis tight;
xlim([0 len])

subplot (434) % right C7shoulder x component
plot1 = plot(tau.estimated.rightC7Shoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5);
hold on
ylabel('rightC7Shoulder','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (435) % right C7shoulder y component
plot1 = plot(tau.estimated.rightShoulder.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('y');
grid on;
axis tight;
xlim([0 len])

subplot (436) % right C7shoulder z component
plot1 = plot(tau.estimated.rightShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('z');
grid on;
axis tight;
xlim([0 len])

subplot (437) % right elbow x component
plot1 = plot(tau.estimated.rightElbow.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5);
hold on
ylabel('rightElbow','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (438) % right elbow y component
plot1 = plot(tau.estimated.rightElbow.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('y');
grid on;
axis tight;
xlim([0 len])

subplot (439) % right elbow z component
plot1 = plot(tau.estimated.rightElbow.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('z');
grid on;
axis tight;
xlim([0 len])


subplot (4,3,10) % right wrist x component
plot1 = plot(tau.estimated.rightWrist.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5);
hold on
ylabel('rightWrist','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (4,3,11) % right wrist y component
plot1 = plot(tau.estimated.rightWrist.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('y');
grid on;
axis tight;
xlim([0 len])

subplot (4,3,12) % right wrist z component
plot1 = plot(tau.estimated.rightWrist.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('z');
grid on;
axis tight;
xlim([0 len])

% % % tau plot legend
% % leg = legend([plot1],{'estim'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

% % put a title on the top of the subplots
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('tau_dx_arm')),fig,600);
end

%% LEFT ARM
fig = figure('Name', 'Left arm torques','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

subplot (431) % left shoulder x component
plot1 = plot(tau.estimated.leftShoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftShoulder.rotx,2.*sqrt(Sigma_tau.rightShoulder_rotx),'b',1.5);
hold on
ylabel('leftShoulder','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('N samples');
title ('x');
grid on;
axis tight;
xlim([0 len])

subplot (432) % left shoulder y component
plot1 = plot(tau.estimated.leftShoulder.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.roty,2.*sqrt(Sigma_tau.rightShoulder_roty),'b',1.5);
xlabel('N samples');
title ('y');
grid on;
axis tight;
xlim([0 len])

subplot (433) % left shoulder z component
plot1 = plot(tau.estimated.leftShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('z');
grid on;
axis tight;
xlim([0 len])

subplot (434) % left C7shoulder x component
plot1 = plot(tau.estimated.leftC7Shoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5);
hold on
ylabel('rightC7Shoulder','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (435) % left C7shoulder y component
plot1 = plot(tau.estimated.leftShoulder.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('y');
grid on;
axis tight;
xlim([0 len])

subplot (436) % left C7shoulder z component
plot1 = plot(tau.estimated.leftShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('z');
grid on;
axis tight;
xlim([0 len])

subplot (437) % left elbow x component
plot1 = plot(tau.estimated.leftElbow.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5);
hold on
ylabel('rightElbow','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (438) % left elbow y component
plot1 = plot(tau.estimated.leftElbow.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('y');
grid on;
axis tight;
xlim([0 len])

subplot (439) % left elbow z component
plot1 = plot(tau.estimated.leftElbow.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('z');
grid on;
axis tight;
xlim([0 len])


subplot (4,3,10) % left wrist x component
plot1 = plot(tau.estimated.leftWrist.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5);
hold on
ylabel('rightWrist','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('N samples');
grid on;
axis tight;
xlim([0 len])

subplot (4,3,11) % right wrist y component
plot1 = plot(tau.estimated.leftWrist.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('y');
grid on;
axis tight;
xlim([0 len])

subplot (4,3,12) % left wrist z component
plot1 = plot(tau.estimated.leftWrist.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('N samples');
title ('z');
grid on;
axis tight;
xlim([0 len])

% % % tau plot legend
% % leg = legend([plot1],{'estim'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

% % put a title on the top of the subplots
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('tau_sx_arm')),fig,600);
end
