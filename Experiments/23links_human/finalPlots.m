
if opts.EXO
    if opts.EXO_torqueLevelAnalysis
        % -----------------------------------------------------------------
        % ==================== EXO_torqueLevelAnalysis=====================
        % -----------------------------------------------------------------
        
        % ------------Right Shoulder
        fig = figure('Name', 'EXO_torqueLevelAnalysis','NumberTitle','off');
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        
        for blockIdx = 1 : block.nrOfBlocks
            subplot (5,1,blockIdx)
            % MAPtorque after CoC --> tauFirst_MAPest
            plot1 = plot(CoC(blockIdx).Rsho_tauFirst(1,:),'lineWidth',1.5);
            hold on;
            % EXOtableTorque --> tau_EXOfromTable
            plot2 = plot(exo_tauLevel(blockIdx).torqueFromTable_right,'lineWidth',1.5);
            hold on;
            % tau_final --> tauFirst_MAPest - tau_EXOfromTable
            plot3 = plot(exo_tauLevel(blockIdx).finalTorque_right,'lineWidth',1.5);
            title(sprintf('Right Shoulder EXO, Block %s', num2str(blockIdx)));
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend([plot1,plot2,plot3],{'$\tau\prime_{MAPest}$','$\tau_{EXO}$', ...
                '$\tau\prime_{MAPest}-\tau_{EXO}$'},'Location','northeast');
            set(leg,'Interpreter','latex');
        end
        % ------------Left Shoulder
        fig = figure('Name', 'EXO_torqueLevelAnalysis','NumberTitle','off');
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        
        for blockIdx = 1 : block.nrOfBlocks
            subplot (5,1,blockIdx)
            % MAPtorque after CoC --> tauFirst_MAPest
            plot1 = plot(CoC(blockIdx).Lsho_tauFirst(1,:),'lineWidth',1.5);
            hold on;
            % EXOtableTorque --> tau_EXOfromTable
            plot2 = plot(exo_tauLevel(blockIdx).torqueFromTable_left,'lineWidth',1.5);
            hold on;
            % tau_final --> tauFirst_MAPest - tau_EXOfromTable
            plot3 = plot(exo_tauLevel(blockIdx).finalTorque_left,'lineWidth',1.5);
            title(sprintf('Left Shoulder EXO, Block %s', num2str(blockIdx)));
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend([plot1,plot2,plot3],{'$\tau\prime_{MAPest}$', ...
                '$\tau\prime_{EXO}$','$\tau\prime_{MAPest}-\tau_{EXO}$'},'Location','northeast');
            set(leg,'Interpreter','latex');
        end
    end
    
    if opts.EXO_forceLevelAnalysis
        % ---------------------------------------------------------------------
        % ====================== EXO_forceLevelAnalysis =======================
        % ---------------------------------------------------------------------
        % to be written
    end
else
    % ---------------------------------------------------------------------
    % =========================== NO EXO Analysis =========================
    % ---------------------------------------------------------------------
    
    % ------------Right Shoulder
    fig = figure('Name', 'noEXO analysis','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    
    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % tau MAPest without no exo --> tauFirst_MAPest
        plot1 = plot(CoC(blockIdx).Rsho_tauFirst(1,:),'lineWidth',1.5);
        title(sprintf('Right Shoulder NO EXO, Block %s', num2str(blockIdx)));
        ylabel('torque [Nm]');
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend(plot1,{'$\tau\prime_{MAPest}$'},'Location','northeast');
        set(leg,'Interpreter','latex');
    end
    % ------------Left Shoulder
    fig = figure('Name', 'noEXO analysis','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    
    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % tau MAPest without no exo --> tauFirst_MAPest
        plot1 = plot(CoC(blockIdx).Lsho_tauFirst(1,:),'lineWidth',1.5);
        title(sprintf('Left Shoulder NO EXO, Block %s', num2str(blockIdx)));
        ylabel('torque [Nm]');
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend(plot1,{'$\tau\prime_{MAPest}$'},'Location','northeast');
        set(leg,'Interpreter','latex');
    end
end
