close all
range_cut_plot = (27:1082);
mu_dgiveny_remove_LeftHand = mu_dgiveny_remove_LeftHand(:,range_cut_plot);
y = y(:,range_cut_plot);

len = size(mu_dgiveny_remove_LeftHand,2);
specific_vector_sigma = zeros(1,len);

figFolder = fullfile(pwd,'Figures');
if(exist(figFolder,'dir')==0)
    mkdir(figFolder);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  EXTERNAL FORCES REMOVING SENSORS
% -----------------------------------------------------------------------%
fig = figure();

% fext.measured in y vector (link frame)
range_fextMEAS_rightFoot = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'RightFoot');
fext.measured.rightFoot = y((range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5),:);
fext.measured.rightFoot_sigma = diag(Sigmay((range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5),(range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5)));

% fext.estimated (after MAP) 
range_fextEST_rightFoot = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'RightFoot');
fext.estimated.rightFoot = mu_dgiveny_remove_LeftHand((range_fextEST_rightFoot:range_fextEST_rightFoot+5 ),:);

subplot (331) % Right foot x component
plot1 = plot(fext.estimated.rightFoot(1,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(1);
shad2 = shadedErrorBar([],fext.measured.rightFoot(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1); 
ylabel(' rightFoot','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',12,...
       'Interpreter','latex');
xlim([0 len])
title ('x');
grid on;

subplot (332) % Right foot y component
plot1 = plot(fext.estimated.rightFoot(2,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(2);
shad2 = shadedErrorBar([],fext.measured.rightFoot(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1); 
xlim([0 len])
title ('x');
grid on;

subplot (333) % Right foot z component
plot1 = plot(fext.estimated.rightFoot(3,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(3);
shad2 = shadedErrorBar([],fext.measured.rightFoot(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1); 
xlim([0 len])
title ('x');
grid on;

% % -------------
% fext.measured in y vector (link frame)
range_fextMEAS_rightHand = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'RightHand');
fext.measured.rightHand = y((range_fextMEAS_rightHand:range_fextMEAS_rightHand+5),:);
fext.measured.rightHand_sigma = diag(Sigmay((range_fextMEAS_rightHand:range_fextMEAS_rightHand+5),(range_fextMEAS_rightHand:range_fextMEAS_rightHand+5)));

% fext.estimated (after MAP) 
range_fextEST_rightHand = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'RightHand');
fext.estimated.rightHand = mu_dgiveny_remove_LeftHand((range_fextEST_rightHand:range_fextEST_rightHand+5 ),:);

subplot (334) % rightHand x component
plot1 = plot(fext.estimated.rightHand(1,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(1);
shad2 = shadedErrorBar([],fext.measured.rightHand(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1); 
ylabel(' rightHand','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',12,...
       'Interpreter','latex');
xlim([0 len])
title ('x');
grid on;

subplot (335) % rightHand y component
plot1 = plot(fext.estimated.rightHand(2,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(2);
shad2 = shadedErrorBar([],fext.measured.rightHand(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1); 
xlim([0 len])
title ('x');
grid on;

subplot (336) % rightHand z component
plot1 = plot(fext.estimated.rightHand(3,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(3);
shad2 = shadedErrorBar([],fext.measured.rightHand(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1); 
xlim([0 len])
title ('x');
grid on;


% % -------------
% fext.measured in y vector (link frame)
range_fextMEAS_leftHand = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'LeftHand');
fext.measured.leftHand = y((range_fextMEAS_leftHand:range_fextMEAS_leftHand+5),:);
fext.measured.leftHand_sigma = diag(Sigmay((range_fextMEAS_leftHand:range_fextMEAS_leftHand+5),(range_fextMEAS_leftHand:range_fextMEAS_leftHand+5)));

% fext.estimated (after MAP) 
range_fextEST_leftHand = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'LeftHand');
fext.estimated.leftHand = mu_dgiveny_remove_LeftHand((range_fextEST_leftHand:range_fextEST_leftHand+5 ),:);

subplot (337) % leftHand x component
plot1 = plot(fext.estimated.leftHand(1,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(1);
shad2 = shadedErrorBar([],fext.measured.leftHand(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1); 
ylabel(' leftHand','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',12,...
       'Interpreter','latex');
xlim([0 len])
title ('x');
grid on;

subplot (338) % leftHand y component
plot1 = plot(fext.estimated.leftHand(2,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(2);
shad2 = shadedErrorBar([],fext.measured.leftHand(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1); 
xlim([0 len])
title ('x');
grid on;

subplot (339) % leftHand z component
plot1 = plot(fext.estimated.leftHand(3,:),'b','lineWidth',1.5);
hold on 
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(3);
shad2 = shadedErrorBar([],fext.measured.leftHand(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1); 
xlim([0 len])
title ('x');
grid on;

% % -------------
% fext plot legend
leg = legend([plot1,shad2.mainLine],{'estimated','measured'},'Location','northeast');
set(leg,'Interpreter','latex');
set(leg,'FontSize',13);

% put a title on the top of the subplots
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
text(0.5, 1,'\bf External force [N] _removing LeftHand FTs','HorizontalAlignment','center','VerticalAlignment', 'top');

