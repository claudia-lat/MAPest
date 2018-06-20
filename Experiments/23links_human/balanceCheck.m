
close all;
%% Preliminaries
% % Mass of the entire shoes
% mass.completeShoe = 1.17; % mass of each shoe
% mass.noSensShoe   = 0.50; % mass of the each shoe with only the sandal + the two attached plates. No FTs or lowest plates!

% Compute the 'weight' of the equipment from FP data
staticWeightFP_mean = mean(synchroData(1).FP_SF(1:400,3)); % first 400 samples
mass.equipmentFP = (staticWeightFP_mean/9.81) - masterFile.Subject.Info.Weight;
% since equipmentMassFP is computed from the forceplates, it considers the
% weight of: entire shoes(1.17kg x2) + suit(1kg) + drill(0.66kg) +
% respiratory stuff(?kg)

% Compute the 'weight' of the equipment from FtShoes data
staticWeightFtShoes_mean = mean(data(blockIdx).data(84).meas(3,1:400)) + mean(data(blockIdx).data(76).meas(3,1:400)); % first 400 samples
mass.equipmentFtShoes = (staticWeightFtShoes_mean/9.81) - masterFile.Subject.Info.Weight;
% since equipmentMassFtShoes is computed from the ftShoes, it considers the
% weight of: part of the shoe + suit(1kg) + drill(0.66kg) +
% respiratory stuff(?kg)

% Compute rotation matrices for MAP transform
for linkIdx = 1: size(suit.links,1)
    suit_test.links{linkIdx, 1}.label        = suit.links{linkIdx, 1}.label;
    for blockIdx = 1 : block.nrOfBlocks
        suit_test.links{linkIdx, 1}.meas(blockIdx).block  = block.labels(blockIdx);
        suit_test.links{linkIdx, 1}.meas(blockIdx).orientation = suit.links{linkIdx, 1}.meas.orientation(:,tmp.cutRange{blockIdx});
        
        suit_test.links{linkIdx, 1}.meas(blockIdx).G_R_L = cell(size(suit.links{linkIdx, 1}.meas.orientation(:,tmp.cutRange{blockIdx}),2), 1);
        for blockLenIdx = 1: size(suit.links{linkIdx, 1}.meas.orientation(:,tmp.cutRange{blockIdx}),2)
            suit_test.links{linkIdx, 1}.meas(blockIdx).G_R_L{blockLenIdx} = quat2Mat(suit_test.links{linkIdx, 1}.meas(blockIdx).orientation(:,blockLenIdx));
        end
    end
end

%% Comparison contact force f_z
rot_RH = iDynTree.Rotation();
rot_RH.RPY(suit.sensors{7, 1}.RPY(1), suit.sensors{7, 1}.RPY(2), suit.sensors{7, 1}.RPY(3));
L_R_S_RH = rot_RH.toMatlab();

rot_LH = iDynTree.Rotation();
rot_LH.RPY(suit.sensors{11, 1}.RPY(1), suit.sensors{11, 1}.RPY(2), suit.sensors{11, 1}.RPY(3));
L_R_S_LH = rot_RH.toMatlab();

for blockIdx = 1 : block.nrOfBlocks
    
    % WITH SHOES
    % contactFz = - (Fz_rightFoot) - (Fz_leftFoot) + (mass)*9.81
    balance(blockIdx).contactFz_ftShoes = - data(blockIdx).data(84).meas(3,:) ...
        - data(blockIdx).data(76).meas(3,:) ...
        + (masterFile.Subject.Info.Weight + mass.equipmentFtShoes)*9.81;
    
    % WITH FORCEPLATES
    % contactFz = - (Fz_fp) + (mass)*9.81
    balance(blockIdx).contactFz_fp = - synchroData(blockIdx).FP_SF(:,3) ...
        + (masterFile.Subject.Info.Weight + mass.equipmentFP)*9.81;
    
    % FROM MAP --> tmp analysis to consider the z component for the balance.
    % The external force is here retrieved NOT for the direct estimation of
    % the MAP, but from the y_sim analysis.
    G_fext_y_sim_RH_tmp = zeros(3,size(suit_test.links{11, 1}.meas(blockIdx).G_R_L,1));
    G_fext_y_sim_LH_tmp = zeros(3,size(suit_test.links{15, 1}.meas(blockIdx).G_R_L,1));
    for blockLenIdx = 1 : size(suit_test.links{11, 1}.meas(blockIdx).G_R_L,1)
        % the forces are here transformed in the global frame G because we
        % know that the z component is pointing up.
        
        % RightHand
        G_fext_y_sim_RH_tmp(:,blockLenIdx) = suit_test.links{11, 1}.meas(blockIdx).G_R_L{blockLenIdx} * L_R_S_RH * y_sim(blockIdx).Fext_RightHand(1:3,blockLenIdx);
        % Left Hand
        G_fext_y_sim_LH_tmp(:,blockLenIdx) = suit_test.links{15, 1}.meas(blockIdx).G_R_L{blockLenIdx} * L_R_S_LH * y_sim(blockIdx).Fext_LeftHand(1:3,blockLenIdx);
    end
    balance(blockIdx).G_fext_y_sim_RH = G_fext_y_sim_RH_tmp;
    balance(blockIdx).G_fext_y_sim_LH = G_fext_y_sim_LH_tmp;
    balance(blockIdx).G_fext_y_sim_hands = balance(blockIdx).G_fext_y_sim_RH + balance(blockIdx).G_fext_y_sim_LH;
end

%% Comparison plot
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

% list_axes = {'x';'y';'z'};
for blockIdx = 1 : block.nrOfBlocks
    subplot (5,1,blockIdx)
    % contact force estimated by MAP
    plot1 = plot(balance(blockIdx).G_fext_y_sim_hands(3,:),'lineWidth',1.5);
    hold on;
    % contact force from balance check WITH SHOES
    plot2 = plot(balance(blockIdx).contactFz_ftShoes,'lineWidth',1.5);
    hold on;
    % contact force from balance check WITH FORCEPLATES
    plot3 = plot(balance(blockIdx).contactFz_fp,'lineWidth',1.5);
    title(sprintf('Block %s', num2str(blockIdx)));
    ylabel('f_z');
    set(gca,'FontSize',15)
    grid on;
    %legend
    leg = legend([plot1,plot2,plot3],{'f_z,MAP','f_z,SHOES','f_z,FP'},'Location','northeast');
    set(leg,'Interpreter','latex');
end

%% fp

% % Comparison plot
% fig = figure();
% axes1 = axes('Parent',fig,'FontSize',16);
% box(axes1,'on');
% hold(axes1,'on');
% grid on;
%
% % list_axes = {'x';'y';'z'};
% for blockIdx = 1 : block.nrOfBlocks
%      subplot (5,1,blockIdx)
%      % contact force estimated by MAP
%      plot1 = plot(masterFile.Subject.FP(blockIdx).FPC(:,3),'lineWidth',1.5);
%      hold on;
%      title(sprintf('Block %s', num2str(blockIdx)));
%      ylabel('f_z');
%      set(gca,'FontSize',15)
%      grid on;
%      %legend
%      leg = legend([plot1],{'fp_z'},'Location','northeast');
%      set(leg,'Interpreter','latex');
% end

%% angles plots

% % fig = figure();
% % axes1 = axes('Parent',fig,'FontSize',16);
% % box(axes1,'on');
% % hold(axes1,'on');
% % grid on;

%% RightShoulder
% %      % contact force estimated by MAP
% %      plot1 = plot(human_state_tmp.q(16,:),'lineWidth',1.5);
% %      hold on;
% %      plot2 = plot(human_state_tmp.q(17,:),'lineWidth',1.5);
% %      hold on;
% %      plot3 = plot(human_state_tmp.q(18,:),'lineWidth',1.5);
% %      ylabel('f_z');
% %      set(gca,'FontSize',15)
% %      grid on;
% %      %legend
% %      leg = legend([plot1, plot2, plot3],{'rightSho_x','rightSho_y','rightSho_z'},'Location','northeast');
% %      set(leg,'Interpreter','latex');

% for blockIdx = 1 : block.nrOfBlocks
%      subplot (5,1,blockIdx)
%      % contact force estimated by MAP
%      plot1 = plot(synchroData(blockIdx).q(16,:),'lineWidth',1.5);
%      hold on;
%      plot2 = plot(synchroData(blockIdx).q(17,:),'lineWidth',1.5);
%      hold on;
%      plot3 = plot(synchroData(blockIdx).q(18,:),'lineWidth',1.5);
%      title(sprintf('Block %s', num2str(blockIdx)));
%      ylabel('f_z');
%      set(gca,'FontSize',15)
%      grid on;
%      %legend
%      leg = legend([plot1, plot2, plot3],{'rightSho_x','rightSho_y','rightSho_z'},'Location','northeast');
%      set(leg,'Interpreter','latex');
% end

% fig = figure();
% axes1 = axes('Parent',fig,'FontSize',16);
% box(axes1,'on');
% hold(axes1,'on');
% grid on;
%
% % LeftShoulder
% for blockIdx = 1 : block.nrOfBlocks
%     subplot (5,1,blockIdx)
%     % contact force estimated by MAP
%     plot1 = plot(synchroData(blockIdx).q(24,:),'lineWidth',1.5);
%     hold on;
%     plot2 = plot(synchroData(blockIdx).q(25,:),'lineWidth',1.5);
%     hold on;
%     plot3 = plot(synchroData(blockIdx).q(26,:),'lineWidth',1.5);
%     title(sprintf('Block %s', num2str(blockIdx)));
%     ylabel('f_z');
%     set(gca,'FontSize',15)
%     grid on;
%     %legend
%     leg = legend([plot1, plot2, plot3],{'rightSho_x','rightSho_y','rightSho_z'},'Location','northeast');
%     set(leg,'Interpreter','latex');
% end
