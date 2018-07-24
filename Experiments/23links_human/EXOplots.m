
%% PLOTS EXO TORQUES
%% PLOTS section
if EXO.opts.plots
    % ------------Current VS fixed frame, Right Shoulder-------------------
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;

    blockIdx = 3;
    subplot (211)
    plot(EXO.jRightShoulder_rotx_grad,'r','Linewidth',1.5)
    hold on
    plot(EXO.jRightShoulder_roty_grad,'g','Linewidth',1.5)
    hold on
    plot(EXO.jRightShoulder_rotz_grad,'b','Linewidth',1.5)
    ylabel('joint angle [deg]','FontSize',15);
    % xlabel('samples','FontSize',15);
    title(sprintf('Right CURRENT FRAME, Block %s', num2str(blockIdx)))
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)

    subplot (212)
    plot(EXO.RightRpy_deg(1,:),'r','Linewidth',1.5)
    hold on
    plot(EXO.RightRpy_deg(2,:),'g','Linewidth',1.5)
    hold on
    plot(EXO.RightRpy_deg(3,:),'b','Linewidth',1.5)
    ylabel('joint angle [deg]','FontSize',15);
    xlabel('samples','FontSize',15);
    title(sprintf('Right  FIXED FRAME, Block %s', num2str(blockIdx)))
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)

    % ------------Current VS fixed frame, Left Shoulder--------------------
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');

    blockIdx = 3;
    subplot (211)
    plot(EXO.jLeftShoulder_rotx_grad,'r','Linewidth',1.5)
    hold on
    plot(EXO.jLeftShoulder_roty_grad,'g','Linewidth',1.5)
    hold on
    plot(EXO.jLeftShoulder_rotz_grad,'b','Linewidth',1.5)
    ylabel('joint angle [deg]','FontSize',15);
    % xlabel('samples','FontSize',15);
    title(sprintf('Left CURRENT FRAME, Block %s', num2str(blockIdx)))
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)

    subplot (212)
    plot(EXO.LeftRpy_deg(1,:),'r','Linewidth',1.5)
    hold on
    plot(EXO.LeftRpy_deg(2,:),'g','Linewidth',1.5)
    hold on
    plot(EXO.LeftRpy_deg(3,:),'b','Linewidth',1.5)
    ylabel('joint angle [deg]','FontSize',15);
    xlabel('samples','FontSize',15);
    title(sprintf('Left FIXED FRAME, Block %s', num2str(blockIdx)))
    leg = legend('rotx','roty','rotz');
    set(leg,'FontSize',17)

    % ------------Final torque, Right Shoulder-----------------------------
    fig = figure();
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;

    for blockIdx = 1 : block.nrOfBlocks
        subplot (5,1,blockIdx)
        % tau MAPest without any exo consideration
        plot1 = plot(estimatedVariables.tau(blockIdx).values(EXO.jRshoRotx_idx,:),'lineWidth',1.5);
        hold on;
        % tau from EXO tab
        plot2 = plot(exo(blockIdx).torqueFromTable_right,'lineWidth',1.5);
        hold on;
        % tau_diff
        plot3 = plot(exo(blockIdx).torqueDiff_right,'lineWidth',1.5);
        title(sprintf('Right, Block %s', num2str(blockIdx)));
        ylabel('torque');
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2,plot3],{'$\tau_{MAPest}$','$\tau_{EXO}$','$\tau_{diff}$'},'Location','northeast');
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
        % tau MAPest without any exo consideration
        plot1 = plot(estimatedVariables.tau(blockIdx).values(EXO.jLshoRotx_idx,:),'lineWidth',1.5);
        hold on;
        % tau from EXO tab
        plot2 = plot(exo(blockIdx).torqueFromTable_left,'lineWidth',1.5);
        hold on;
        % tau_diff
        plot3 = plot(exo(blockIdx).torqueDiff_left,'lineWidth',1.5);
        title(sprintf('Left, Block %s', num2str(blockIdx)));
        ylabel('torque');
        set(gca,'FontSize',15)
        grid on;
        %legend
        leg = legend([plot1,plot2,plot3],{'$\tau_{MAPest}$','$\tau_{EXO}$','$\tau_{diff}$'},'Location','northeast');
        set(leg,'Interpreter','latex');
    end
end