
%% PLOTS EXO TORQUES
close all;

if EXO.opts.plots
    % ------------Final torque, Right Shoulder-----------------------------
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;

    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % tau MAPest without any exo consideration --> tauFirst_MAPest
        plot1 = plot(exo(blockIdx).Rsho_tauFirst(1,:),'lineWidth',1.5);
        hold on;
        % tau from EXO tab --> tau_EXO
        plot2 = plot(exo(blockIdx).torqueFromTable_right,'lineWidth',1.5);
        hold on;
        % tau_diff --> tauFirst_MAPest - tau_EXO
        plot3 = plot(exo(blockIdx).torqueDiff_right,'lineWidth',1.5);
        title(sprintf('Right Shoulder, Block %s', num2str(blockIdx)));
        ylabel('torque [Nm]');
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2,plot3],{'$\tau\prime_{MAPest}$','$\tau_{EXO}$','$\tau\prime_{MAPest}-\tau_{EXO}$'},'Location','northeast');
        set(leg,'Interpreter','latex');
    end
    % ------------Final torque, Left Shoulder------------------------------
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    
    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % tau MAPest without any exo consideration --> tauFirst_MAPest
        plot1 = plot(exo(blockIdx).Lsho_tauFirst(1,:),'lineWidth',1.5);
        hold on;
        % tau from EXO tab --> tau_EXO
        plot2 = plot(exo(blockIdx).torqueFromTable_left,'lineWidth',1.5);
        hold on;
        % tau_diff --> tauFirst_MAPest - tau_EXO
        plot3 = plot(exo(blockIdx).torqueDiff_left,'lineWidth',1.5);
        title(sprintf('Left Shoulder, Block %s', num2str(blockIdx)));
        ylabel('torque [Nm]');
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2,plot3],{'$\tau\prime_{MAPest}$','$\tau\prime_{EXO}$','$\tau\prime_{MAPest}-\tau_{EXO}$'},'Location','northeast');
        set(leg,'Interpreter','latex');
    end
end
