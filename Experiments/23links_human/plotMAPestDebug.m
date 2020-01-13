
close all

if ~opts.task1_SOT
    
    %% 6D accelerations plots
    % for Idx = 1  : length(dVectorOrder)
    %     fig = figure('Name', '6D acceleration - ESTIMATION','NumberTitle','off');
    %     axes1 = axes('Parent',fig,'FontSize',16);
    %     box(axes1,'on');
    %     hold(axes1,'on');
    %     grid on;
    %
    %     plot1 = plot(estimatedVariables.Acc.values(6*(Idx-1)+1:6*Idx,:)','lineWidth',1.5);
    %     title(sprintf('%s',dVectorOrder{Idx,1}),'Interpreter','latex');
    %     leg = legend(plot1,{'$alin_x$','$alin_y$','$alin_z$','$aang_x$','$aang_y$','$aang_z$',},'Location','northeast');
    %     set(leg,'Interpreter','latex');
    % %     pause
    % end
    
    %% External wrenches plots
    % for Idx = 1  : length(dVectorOrder)
    %     fig = figure('Name', '6D external force - ESTIMATION','NumberTitle','off');
    %     axes1 = axes('Parent',fig,'FontSize',16);
    %     box(axes1,'on');
    %     hold(axes1,'on');
    %     grid on;
    %
    %     plot1 = plot(estimatedVariables.Fext.values(6*(Idx-1)+1:6*Idx,:)','lineWidth',1.5);
    %     title(sprintf('%s',dVectorOrder{Idx,1}),'Interpreter','latex');
    %     leg = legend(plot1,{'$f_x$','$f_y$','$f_z$','$m_x$','$m_y$','$m_z$',},'Location','northeast');
    %     set(leg,'Interpreter','latex');
    % %     pause
    % end
    
    
    %% Estimated torques
    fig = figure('Name', 'torque - ESTIMATION','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    
    for nrDofsIdx = 1  : nrDofs
        subplot (5,10,nrDofsIdx)
        plot1 = plot(estimatedVariables.tau.values(nrDofsIdx,:),'m','lineWidth',1);
        
        title(sprintf('%s',estimatedVariables.tau.label{nrDofsIdx, 1}));
    end
    
    %% Estimated 6D internal forces
    fig = figure('Name', '6D internal wrench - ESTIMATION','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    
    for nrDofsIdx = 1  : nrDofs
        subplot (5,10,nrDofsIdx)
        %  3D forces
        plot1 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+1,:)','r','lineWidth',1.5);
        hold on
        plot2 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+2,:)','g','lineWidth',1.5);
        hold on
        plot3 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+3,:)','b','lineWidth',1.5);
        % 3D moments
        plot5 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+4,:)','lineWidth',1.5);
        hold on
        plot6 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+5,:)','lineWidth',1.5);
        hold on
        plot7 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+6,:)','lineWidth',1.5);
        title(sprintf('%s',estimatedVariables.Fint.label{nrDofsIdx, 1}));
    end
    
    %% Estimated internal 3D momentss
    % fig = figure('Name', '3D internal moment - ESTIMATION','NumberTitle','off');
    % axes1 = axes('Parent',fig,'FontSize',16);
    % box(axes1,'on');
    % hold(axes1,'on');
    %
    % for nrDofsIdx = 1  : nrDofs
    %     subplot (5,10,nrDofsIdx)
    %     %     plot1 = plot(estimatedVariables.Fint.values(nrDofsIdx,:),'m','lineWidth',1);
    %     plot1 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+4,:)','r','lineWidth',1.5);
    %     hold on
    %     plot2 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+5,:)','g','lineWidth',1.5);
    %     hold on
    %     plot3 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+6,:)','b','lineWidth',1.5);
    %
    % %     plot1 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+1 : 6*nrDofsIdx-3,:)','r','lineWidth',1.5);
    % %     hold on
    % %     plot2 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+2 : 6*nrDofsIdx-2,:)','g','lineWidth',1.5);
    % %     hold on
    % %     plot3 = plot(estimatedVariables.Fint.values(6*(nrDofsIdx-1)+3 : 6*nrDofsIdx-1,:)','b','lineWidth',1.5);
    %     title(sprintf('%s',estimatedVariables.Fint.label{nrDofsIdx, 1}));
    % end
    
    %%
    % -----------------------------------------------------------------------%
    %  LINEAR ACCELERATION
    % -----------------------------------------------------------------------%
    fig = figure('Name', '3D linear acceleration - MEAS vs. ESTIM','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    
    for linAccIdx = 1  : nrOfLinAccelerometer
        subplot (3,6,linAccIdx)
        % from the measurement
        plot1 = plot(data(linAccIdx).meas(1,:),'r','lineWidth',1.5);
        plot2 = plot(data(linAccIdx).meas(2,:),'g','lineWidth',1.5);
        plot3 = plot(data(linAccIdx).meas(3,:),'b','lineWidth',1.5);
        hold on
        % from the estimation (y_sim)
        plot7  = plot(y_sim_linAcc.meas{linAccIdx,1}(1,:),'r--o','lineWidth',0.5);
        plot8  = plot(y_sim_linAcc.meas{linAccIdx,1}(2,:),'g--o','lineWidth',0.5);
        plot9  = plot(y_sim_linAcc.meas{linAccIdx,1}(3,:),'b--o','lineWidth',0.5);
        grid on;
        
        title(sprintf('%s',y_sim_linAcc.order{linAccIdx, 1}));
        
        %     % fext plot legend
        % %     leg = legend([plot1,plot2],{'estim','meas'},'Location','northeast');
        % %     set(leg,'Interpreter','latex', ...
        % %         'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
        % %         'Orientation','horizontal');
        % %     set(leg,'FontSize',13);
        %
    end
    
    %%
    % -----------------------------------------------------------------------%
    %  JOINT ACCELERATION
    % -----------------------------------------------------------------------%
    % fig = figure('Name', 'joint acceleration - MEAS vs. ESTIM','NumberTitle','off');
    % axes1 = axes('Parent',fig,'FontSize',16);
    % box(axes1,'on');
    % hold(axes1,'on');
    %
    % for nrDofsIdx = 1  : nrDofs
    %     subplot (5,10,nrDofsIdx)
    %     % from the measurement
    %     plot1 = plot(synchroKin.ddq(nrDofsIdx,:),'k','lineWidth',1);
    %     hold on
    %     % from the estimation (y_sim)
    %     plot2 = plot(y_sim_ddq.meas{nrDofsIdx},'m','lineWidth',1);
    %
    %     title(sprintf('%s',y_sim_ddq.order{nrDofsIdx, 1}));
    % end
    
    
    %% dq,ddq
    % fig = figure();
    % axes1 = axes('Parent',fig,'FontSize',16);
    % box(axes1,'on');
    % hold(axes1,'on');
    %
    % for nrDofsIdx = 1  : nrDofs
    %     subplot (5,10,nrDofsIdx)
    %     plot0 = plot(synchroKin.state.q(nrDofsIdx,:),'b','lineWidth',1);
    %     hold on
    %     plot1 = plot(synchroKin.state.dq(nrDofsIdx,:),'k','lineWidth',1);
    %     hold on
    %     plot2 = plot(synchroKin.ddq(nrDofsIdx,:),'m','lineWidth',1);
    %
    %     title(sprintf('%s',selectedJoints{nrDofsIdx, 1}));
    % end
    
end


%% MEASUREMENTS vs. ESTIMATIONS
% -----------------------------------------------------------------------%
%  EXTERNAL FORCES
% -----------------------------------------------------------------------%
fig = figure('Name', '6D external wrench - MEAS vs. ESTIM','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');

for vectOrderIdx = 1 : length(dVectorOrder)
    subplot (5,10,vectOrderIdx)
    % from the measurement
    for dataFextIdx = 66 : 114
        if strcmp(data(dataFextIdx).id,dVectorOrder{vectOrderIdx})
            plot1 = plot(data(dataFextIdx).meas(1,:),'r','lineWidth',1.5);
            plot2 = plot(data(dataFextIdx).meas(2,:),'g','lineWidth',1.5);
            plot3 = plot(data(dataFextIdx).meas(3,:),'b','lineWidth',1.5);
            %             plot4 = plot(data(dataFextIdx).meas(4,:),'lineWidth',1.5);
            %             plot5 = plot(data(dataFextIdx).meas(5,:),'lineWidth',1.5);
            %             plot6 = plot(data(dataFextIdx).meas(6,:),'lineWidth',1.5);
            break;
        end
    end
    hold on
    % from the estimation (y_sim)
    plot7  = plot(y_sim_fext.meas{vectOrderIdx,1}(1,:),'r--o','lineWidth',0.5);
    plot8  = plot(y_sim_fext.meas{vectOrderIdx,1}(2,:),'g--o','lineWidth',0.5);
    plot9  = plot(y_sim_fext.meas{vectOrderIdx,1}(3,:),'b--o','lineWidth',0.5);
    %     plot10 = plot(y_sim_fext.meas{vectOrderIdx,1}(4,:),'o','lineWidth',1.5);
    %     plot11 = plot(y_sim_fext.meas{vectOrderIdx,1}(5,:),'o','lineWidth',1.5);
    %     plot12 = plot(y_sim_fext.meas{vectOrderIdx,1}(6,:),'o','lineWidth',1.5);
    grid on;
    
    title(sprintf('%s',dVectorOrder{vectOrderIdx,1}));
    
    % fext plot legend
    %       leg = legend([plot1,plot2,plot3, ... plot4,plot5,plot6, ...
    %          plot7,plot8,plot9], ... plot10,plot11,plot12], ...
    %          {'$f_{x,MEAS}$','$f_{y,MEAS}$','$f_{z,MEAS}$', ...%'$m_{x,MEAS}$','$m_{y,MEAS}$','$m_{z,MEAS}$', ...
    %          '$f_{x,ESTIM}$','$f_{y,ESTIM}$','$f_{z,ESTIM}$'},'Location','northeast');%'$m_{x,ESTIM}$','$m_{y,ESTIM}$','$m_{z,ESTIM}$'},'Location','northeast');
    %      set(leg,'Interpreter','latex');
    %     set(leg,'FontSize',13);
    %
    %         if vectOrderIdx == 49
    %             leg = legend([plot1,plot7],{'MEAS','ESTIM'});
    %             set(leg,'Interpreter','latex', ...
    %                 'Orientation','horizontal');
    %             set(leg,'FontSize',13);
    %         end
end

