close all
range_cut_plot = (27:1082);
y_simulated_remove_accRightHand = y_simulated_remove_accRightHand(:,range_cut_plot);


len = size(y_simulated_remove_accRightHand,2);

figFolder = fullfile(pwd,'Figures');
if(exist(figFolder,'dir')==0)
    mkdir(figFolder);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  ACC REMOVING SENSORS
% -----------------------------------------------------------------------%
fig = figure();

% acc.measured in y vector (link frame)
acc.measured.accRightHand = suit.sensors{7, 1}.meas.sensorAcceleration;
acc.measured.accRightHand = acc.measured.accRightHand(:,range_cut_plot);

% acc.estimated (after MAP) 
range_accMEAS_rightHand = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, 'RightHand_accelerometer');
acc.simulated.rightHand = y_simulated_remove_accRightHand((range_accMEAS_rightHand:range_accMEAS_rightHand+5),:);

subplot (131) % RightHand x component
plot1 = plot(acc.estimated.accRightHand(1,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(acc.measured.accRightHand(1,:),'r','lineWidth',1.5);
ylabel(' rightFoot','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',12,...
       'Interpreter','latex');
xlim([0 len])
title ('x');
grid on;

subplot (132) % RightHand y component
plot1 = plot(acc.estimated.accRightHand(2,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(acc.measured.accRightHand(2,:),'r','lineWidth',1.5);
xlim([0 len])
title ('y');
grid on;

subplot (133) % RightHand z component
plot1 = plot(acc.estimated.accRightHand(3,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(acc.measured.accRightHand(3,:),'r','lineWidth',1.5);
% specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(3);
% shad2 = shadedErrorBar([],acc.measured.accRightHand(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1); 
xlim([0 len])
title ('y');
grid on;


% % -------------
% fext plot legend
leg = legend([plot1,plot2],{'estimated','measured'},'Location','northeast');
set(leg,'Interpreter','latex');
set(leg,'FontSize',13);

% put a title on the top of the subplots
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
text(0.5, 1,'\bf Linear acc _removing accelerometer on RightHand','HorizontalAlignment','center','VerticalAlignment', 'top');

