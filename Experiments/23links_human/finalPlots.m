
finalPlot = false;

if finalPlot
    if opts.EXO
        % ------------Final EXO torque, Right Shoulder-------------------------
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        
        for blockIdx = 1 : block.nrOfBlocks
            subplot (5,1,blockIdx)
            % tau MAPest without any exo consideration --> tauFirst_MAPest
            plot1 = plot(CoC(blockIdx).Rsho_tauFirst(1,:),'lineWidth',1.5);
            hold on;
            % tau from EXO tab --> tau_EXOfromTable
            plot2 = plot(exo(blockIdx).torqueFromTable_right,'lineWidth',1.5);
            hold on;
            % tau_diff --> tauFirst_MAPest - tau_EXOfromTable
            plot3 = plot(exo(blockIdx).torqueDiff_right,'lineWidth',1.5);
            title(sprintf('Right Shoulder EXO, Block %s', num2str(blockIdx)));
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend([plot1,plot2,plot3],{'$\tau\prime_{MAPest}$','$\tau_{EXO}$', ...
                '$\tau\prime_{MAPest}-\tau_{EXO}$'},'Location','northeast');
            set(leg,'Interpreter','latex');
        end
        % ------------Final EXO torque, Left Shoulder--------------------------
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        
        for blockIdx = 1 : block.nrOfBlocks
            subplot (5,1,blockIdx)
            % tau MAPest without any exo consideration --> tauFirst_MAPest
            plot1 = plot(CoC(blockIdx).Lsho_tauFirst(1,:),'lineWidth',1.5);
            hold on;
            % tau from EXO tab --> tau_EXOfromTable
            plot2 = plot(exo(blockIdx).torqueFromTable_left,'lineWidth',1.5);
            hold on;
            % tau_diff --> tauFirst_MAPest - tau_EXOfromTable
            plot3 = plot(exo(blockIdx).torqueDiff_left,'lineWidth',1.5);
            title(sprintf('Left Shoulder EXO, Block %s', num2str(blockIdx)));
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend([plot1,plot2,plot3],{'$\tau\prime_{MAPest}$', ...
                '$\tau\prime_{EXO}$','$\tau\prime_{MAPest}-\tau_{EXO}$'},'Location','northeast');
            set(leg,'Interpreter','latex');
        end
    else
        % ------------Final No EXO torque, Right Shoulder----------------------
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        
        for blockIdx = 1 : block.nrOfBlocks
            subplot (5,1,blockIdx)
            % tau MAPest without any exo consideration --> tauFirst_MAPest
            plot1 = plot(CoC(blockIdx).Rsho_tauFirst(1,:),'lineWidth',1.5);
            title(sprintf('Right Shoulder NO EXO, Block %s', num2str(blockIdx)));
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend(plot1,{'$\tau\prime_{MAPest}$'},'Location','northeast');
            set(leg,'Interpreter','latex');
        end
        % ------------Final No EXO torque, Left Shoulder------------------------------
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        
        for blockIdx = 1 : block.nrOfBlocks
            subplot (5,1,blockIdx)
            % tau MAPest without any exo consideration --> tauFirst_MAPest
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
end
