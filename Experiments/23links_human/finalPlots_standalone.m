
close all;

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
        % options
        opts.rightArmPlot = true;  %C7shoulder-shoulder-elbow-wrist
        opts.leftArmPlot  = false;  %C7shoulder-shoulder-elbow-wrist
        opts.torsoPlot    = false;  %L5S1-L4L3-L1T12-T9T8-T1C7-C1head
        opts.rightLegPlot = false;  %hip-knee-ankle-ballFoot
        opts.leftLegPlot  = false;  %hip-knee-ankle-ballFoot
        % $$$$$$$$$$$$$$$$$$
        if opts.rightArmPlot
            rightArmIdx   = (15:1:22);
            for i = 1 : length(rightArmIdx)
                fig = figure('Name', 'EXO_forceLevelAnalysis','NumberTitle','off');
                axes1 = axes('Parent',fig,'FontSize',16);
                box(axes1,'on');
                hold(axes1,'on');
                grid on;
                
                for blockIdx = 1 : block.nrOfBlocks
                    torque_index = rightArmIdx(i);
                    subplot (5,1,blockIdx)
                    % MAP torque
                    plot1 = plot(estimatedVariables.tau(blockIdx).values(torque_index,:),'lineWidth',1.5);
                    hold on;
                    % EXO computed torque
                    plot2 = plot(exo_forceLevel(blockIdx).torqueEXO(torque_index,:),'lineWidth',1.5);
                    hold on;
                    % tau_final --> tau_MAPest - tau_EXO
                    plot3 = plot(exo_forceLevel(blockIdx).torque(torque_index,:),'lineWidth',1.5);
                    title(sprintf('%s, Block %s',selectedJoints{rightArmIdx(i)}, num2str(blockIdx)),'Interpreter','latex');
                    ylabel('torque [Nm]');
                    set(gca,'FontSize',15)
                    grid on;
                    %legend
                    leg = legend([plot1,plot2,plot3],{'$\tau_{MAPest}$','$\tau_{EXO}$', ...
                        '$\tau_{MAPest}-\tau_{EXO}$'},'Location','northeast');
                    set(leg,'Interpreter','latex','FontSize',20);
                end
            end
        end
        % $$$$$$$$$$$$$$$$$$
        if opts.leftArmPlot
            leftArmIdx   = (23:1:30);
            for i = 1 : length(leftArmIdx)
                fig = figure('Name', 'EXO_forceLevelAnalysis','NumberTitle','off');
                axes1 = axes('Parent',fig,'FontSize',16);
                box(axes1,'on');
                hold(axes1,'on');
                grid on;
                
                for blockIdx = 1 : block.nrOfBlocks
                    torque_index = leftArmIdx(i);
                    subplot (5,1,blockIdx)
                    % MAP torque
                    plot1 = plot(estimatedVariables.tau(blockIdx).values(torque_index,:),'lineWidth',1.5);
                    hold on;
                    % EXO computed torque
                    plot2 = plot(exo_forceLevel(blockIdx).torqueEXO(torque_index,:),'lineWidth',1.5);
                    hold on;
                    % tau_final --> tau_MAPest - tau_EXO
                    plot3 = plot(exo_forceLevel(blockIdx).torque(torque_index,:),'lineWidth',1.5);
                    title(sprintf('%s, Block %s',selectedJoints{leftArmIdx(i)}, num2str(blockIdx)),'Interpreter','latex');
                    ylabel('torque [Nm]');
                    set(gca,'FontSize',15)
                    grid on;
                    %legend
                    leg = legend([plot1,plot2,plot3],{'$\tau_{MAPest}$','$\tau_{EXO}$', ...
                        '$\tau_{MAPest}-\tau_{EXO}$'},'Location','northeast');
                    set(leg,'Interpreter','latex','FontSize',20);
                end
            end
        end
        % $$$$$$$$$$$$$$$$$$
        if opts.torsoPlot
            torsoIdx   = (1:1:14);
            for i = 1 : length(torsoIdx)
                fig = figure('Name', 'EXO_forceLevelAnalysis','NumberTitle','off');
                axes1 = axes('Parent',fig,'FontSize',16);
                box(axes1,'on');
                hold(axes1,'on');
                grid on;
                
                for blockIdx = 1 : block.nrOfBlocks
                    torque_index = torsoIdx(i);
                    subplot (5,1,blockIdx)
                    % MAP torque
                    plot1 = plot(estimatedVariables.tau(blockIdx).values(torque_index,:),'lineWidth',1.5);
                    hold on;
                    % EXO computed torque
                    plot2 = plot(exo_forceLevel(blockIdx).torqueEXO(torque_index,:),'lineWidth',1.5);
                    hold on;
                    % tau_final --> tau_MAPest - tau_EXO
                    plot3 = plot(exo_forceLevel(blockIdx).torque(torque_index,:),'lineWidth',1.5);
                    title(sprintf('%s, Block %s',selectedJoints{torsoIdx(i)}, num2str(blockIdx)),'Interpreter','latex');
                    ylabel('torque [Nm]');
                    set(gca,'FontSize',15)
                    grid on;
                    %legend
                    leg = legend([plot1,plot2,plot3],{'$\tau_{MAPest}$','$\tau_{EXO}$', ...
                        '$\tau_{MAPest}-\tau_{EXO}$'},'Location','northeast');
                    set(leg,'Interpreter','latex','FontSize',20);
                end
            end
        end
        % $$$$$$$$$$$$$$$$$$
        if opts.rightLegPlot
            rightLegIdx   = (31:1:39);
            for i = 1 : length(rightLegIdx)
                fig = figure('Name', 'EXO_forceLevelAnalysis','NumberTitle','off');
                axes1 = axes('Parent',fig,'FontSize',16);
                box(axes1,'on');
                hold(axes1,'on');
                grid on;
                
                for blockIdx = 1 : block.nrOfBlocks
                    torque_index = rightLegIdx(i);
                    subplot (5,1,blockIdx)
                    % MAP torque
                    plot1 = plot(estimatedVariables.tau(blockIdx).values(torque_index,:),'lineWidth',1.5);
                    hold on;
                    % EXO computed torque
                    plot2 = plot(exo_forceLevel(blockIdx).torqueEXO(torque_index,:),'lineWidth',1.5);
                    hold on;
                    % tau_final --> tau_MAPest - tau_EXO
                    plot3 = plot(exo_forceLevel(blockIdx).torque(torque_index,:),'lineWidth',1.5);
                    title(sprintf('%s, Block %s',selectedJoints{rightLegIdx(i)}, num2str(blockIdx)),'Interpreter','latex');
                    ylabel('torque [Nm]');
                    set(gca,'FontSize',15)
                    grid on;
                    %legend
                    leg = legend([plot1,plot2,plot3],{'$\tau_{MAPest}$','$\tau_{EXO}$', ...
                        '$\tau_{MAPest}-\tau_{EXO}$'},'Location','northeast');
                    set(leg,'Interpreter','latex','FontSize',20);
                end
            end
        end
        % $$$$$$$$$$$$$$$$$$
        if opts.leftLegPlot
            leftLegIdx   = (40:1:48);
            for i = 1 : length(leftLegIdx)
                fig = figure('Name', 'EXO_forceLevelAnalysis','NumberTitle','off');
                axes1 = axes('Parent',fig,'FontSize',16);
                box(axes1,'on');
                hold(axes1,'on');
                grid on;
                
                for blockIdx = 1 : block.nrOfBlocks
                    torque_index = leftLegIdx(i);
                    subplot (5,1,blockIdx)
                    % MAP torque
                    plot1 = plot(estimatedVariables.tau(blockIdx).values(torque_index,:),'lineWidth',1.5);
                    hold on;
                    % EXO computed torque
                    plot2 = plot(exo_forceLevel(blockIdx).torqueEXO(torque_index,:),'lineWidth',1.5);
                    hold on;
                    % tau_final --> tau_MAPest - tau_EXO
                    plot3 = plot(exo_forceLevel(blockIdx).torque(torque_index,:),'lineWidth',1.5);
                    title(sprintf('%s, Block %s',selectedJoints{leftLegIdx(i)}, num2str(blockIdx)),'Interpreter','latex');
                    ylabel('torque [Nm]');
                    set(gca,'FontSize',15)
                    grid on;
                    %legend
                    leg = legend([plot1,plot2,plot3],{'$\tau_{MAPest}$','$\tau_{EXO}$', ...
                        '$\tau_{MAPest}-\tau_{EXO}$'},'Location','northeast');
                    set(leg,'Interpreter','latex','FontSize',20);
                end
            end
        end
    end
else
    % ---------------------------------------------------------------------
    % =========================== NO EXO Analysis =========================
    % ---------------------------------------------------------------------
    
    for jointsIdx = 1 : length(estimatedVariables.tau(1).label)
        fig = figure('Name', 'noEXO analysis','NumberTitle','off');
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        for blockIdx = 1 : block.nrOfBlocks
            subplot (5,1,blockIdx)
            % tau MAP estimation
            plot1 = plot(estimatedVariables.tau(blockIdx).values(jointsIdx,:),'lineWidth',1.5);
            title(sprintf('%s, Block %s',estimatedVariables.tau(blockIdx).label{jointsIdx, 1}, ...
                num2str(blockIdx)),'Interpreter','latex');
            ylabel('torque [Nm]');
            set(gca,'FontSize',15)
            grid on;
            %legend
            leg = legend(plot1,{'$\tau_{MAPest}$'},'Location','northeast');
            set(leg,'Interpreter','latex');
        end
    end
end
