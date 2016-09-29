% EXTRACTION FROM mu_dgiveny FROM THE VARIABLES FOR EACH LINK

nrOfLink =  traversal.getNrOfVisitedLinks() - 1;
P = struct;
% -------------------------------------------------------------------------
% a
P.a.linAcc = zeros(3*nrOfLink,size(mu_dgiveny,2));
P.a.angVel = zeros(3*nrOfLink,size(mu_dgiveny,2));
for i = 1 : nrOfLink
    P.a.linAcc(3*(i-1)+1:3*i, :) = mu_dgiveny(26*(i-1)+1:26*(i-1)+3, :);
    P.a.angVel(3*(i-1)+1:3*i, :) = mu_dgiveny(26*(i-1)+4:26*(i-1)+6, :);
end

% for j = 1 : nrOfLink
%     axes1 = axes('Parent',figure,'FontSize',16);
%                 box(axes1,'on');
%                 hold(axes1,'on');
%                 grid on;
%                 
%     subplot(211);?
%     plot (P.a.linAcc(3*(j-1)+1:3*j,:)');
%     leg = legend('x','y','z','Location','northeast');
%                 set(leg,'Interpreter','latex');
%                 set(leg,'FontSize',13);
%     xlabel('Frames','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     ylabel('lin acc','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     axis tight;
%     grid on;  
%     title (sprintf('%s', dVectorOrder{j+1}));
%     
%     subplot(212);
%     plot (P.a.angVel(3*(j-1)+1:3*j,:)');
%     leg = legend('x','y','z','Location','northeast');
%                 set(leg,'Interpreter','latex');
%                 set(leg,'FontSize',13);
%     xlabel('Frames','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     ylabel('ang vel','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     axis tight;
%     grid on;  
% end
% -------------------------------------------------------------------------
% f_B
P.f_B.force  = zeros(3*nrOfLink,size(mu_dgiveny,2));
P.f_B.moment = zeros(3*nrOfLink,size(mu_dgiveny,2));
for i = 1 : nrOfLink
    P.f_B.force  (3*(i-1)+1:3*i, :) = mu_dgiveny(26*(i-1)+7:26*(i-1)+9, :);
    P.f_B.moment (3*(i-1)+1:3*i, :) = mu_dgiveny(26*(i-1)+10:26*(i-1)+12, :);
end

% for j = 1 : nrOfLink
%     axes1 = axes('Parent',figure,'FontSize',16);
%                 box(axes1,'on');
%                 hold(axes1,'on');
%                 grid on;
%                 
%     subplot(211);
%     plot (P.f_B.force(3*(j-1)+1:3*j,:)');
%     leg = legend('x','y','z','Location','northeast');
%                 set(leg,'Interpreter','latex');
%                 set(leg,'FontSize',13);
%     xlabel('Frames','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     ylabel('net force','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     axis tight;
%     grid on;  
%     title (sprintf('%s', dVectorOrder{j+1}));
%     
%     subplot(212);
%     plot (P.f_B.moment(3*(j-1)+1:3*j,:)');
%     leg = legend('x','y','z','Location','northeast');
%                 set(leg,'Interpreter','latex');
%                 set(leg,'FontSize',13);
%     xlabel('Frames','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     ylabel('net moment','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     axis tight;
%     grid on;  
% end
% -------------------------------------------------------------------------
% f
P.f.force  = zeros(3*nrOfLink,size(mu_dgiveny,2));
P.f.moment = zeros(3*nrOfLink,size(mu_dgiveny,2));
for i = 1 : nrOfLink
    P.f.force  (3*(i-1)+1:3*i, :) = mu_dgiveny(26*(i-1)+13:26*(i-1)+15, :);
    P.f.moment (3*(i-1)+1:3*i, :) = mu_dgiveny(26*(i-1)+16:26*(i-1)+18, :);
end
% 
% for j = 1 : nrOfLink
%     axes1 = axes('Parent',figure,'FontSize',16);
%                 box(axes1,'on');
%                 hold(axes1,'on');
%                 grid on;
%                 
%     subplot(211);
%     plot (P.f.force(3*(j-1)+1:3*j,:)');
%     leg = legend('x','y','z','Location','northeast');
%                 set(leg,'Interpreter','latex');
%                 set(leg,'FontSize',13);
%     xlabel('Frames','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     ylabel('force','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     axis tight;
%     grid on;  
%     title (sprintf('%s', dVectorOrder{j+1}));
%     
%     subplot(212);
%     plot (P.f.moment(3*(j-1)+1:3*j,:)');
%     leg = legend('x','y','z','Location','northeast');
%                 set(leg,'Interpreter','latex');
%                 set(leg,'FontSize',13);
%     xlabel('Frames','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     ylabel('moment','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     axis tight;
%     grid on;  
% end
%-------------------------------------------------------------------------
% tau
P.tau = zeros(nrOfLink,size(mu_dgiveny,2));
for i = 1 : nrOfLink
    P.tau(i,:) = mu_dgiveny(26*(i-1)+19, :);
   P.tau(i,:) = mu_dgiveny(26*i-7, :);
end

for j = 1 : nrOfLink
    axes1 = axes('Parent',figure,'FontSize',16);
            box(axes1,'on');
            hold(axes1,'on');
            grid on;

    plot (P.tau(j,:));
    xlabel('Frames','HorizontalAlignment','center',...
                       'FontWeight','bold',...
                       'FontSize',15,...
                       'Interpreter','latex');
    ylabel('joint torque','HorizontalAlignment','center',...
                       'FontWeight','bold',...
                       'FontSize',15,...
                       'Interpreter','latex');
    axis tight;
    grid on;  
    title (sprintf(' %s', dJointOrder{j}));
end
% -------------------------------------------------------------------------
% f_ext
P.f_ext.force  = zeros(3*nrOfLink,size(mu_dgiveny,2));
P.f_ext.moment = zeros(3*nrOfLink,size(mu_dgiveny,2));
for i = 1 : nrOfLink
    P.f_ext.force   (3*(i-1)+1:3*i, :) = mu_dgiveny(26*(i-1)+20:26*(i-1)+22, :);
    P.f_ext.moment  (3*(i-1)+1:3*i, :) = mu_dgiveny(26*(i-1)+23:26*(i-1)+25, :);
end
% 
% for j = 1 : nrOfLink
%     axes1 = axes('Parent',figure,'FontSize',16);
%                 box(axes1,'on');
%                 hold(axes1,'on');
%                 grid on;
%                 
%     subplot(211);
%     plot (P.f_ext.force(3*(j-1)+1:3*j,:)');
%     leg = legend('x','y','z','Location','northeast');
%                 set(leg,'Interpreter','latex');
%                 set(leg,'FontSize',13);
%     xlabel('Frames','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     ylabel('ext force','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     axis tight;
%     grid on;  
%     title (sprintf('%s', dVectorOrder{j+1}));
%     
%     subplot(212);
%     plot (P.f_ext.moment(3*(j-1)+1:3*j,:)');
%     leg = legend('x','y','z','Location','northeast');
%                 set(leg,'Interpreter','latex');
%                 set(leg,'FontSize',13);
%     xlabel('Frames','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     ylabel('ext moment','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     axis tight;
%     grid on;  
% end
% -------------------------------------------------------------------------
% ddq
P.ddq = zeros(nrOfLink,size(mu_dgiveny,2));
for i = 1 : nrOfLink
    P.ddq(i,:) = mu_dgiveny(26*i, :);
end

% for j = 1 : nrOfLink
%     axes1 = axes('Parent',figure,'FontSize',16);
%             box(axes1,'on');
%             hold(axes1,'on');
%             grid on;
% 
%     plot (P.ddq(j,:));
%     xlabel('Frames','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     ylabel('ddq','HorizontalAlignment','center',...
%                        'FontWeight','bold',...
%                        'FontSize',15,...
%                        'Interpreter','latex');
%     axis tight;
%     grid on;  
%     title (sprintf(' %s', dJointOrder{j}));
% end


%% 
figure
subplot (211)
plot (P.tau(5,:));
hold on
plot (P.tau(11,:));
    axis tight;
    grid on;  
ylabel('jointTorques')
leg = legend('jLeftAnkle','jLeftHip','Location','southeast');
                    set(leg,'Interpreter','latex');
                    set(leg,'FontSize',13);
title ('Rotation on y axis');                   

subplot(212)
plot(human_state.q(24,:)*180/pi)
hold on
plot(human_state.q(6,:)*180/pi)
axis tight;
grid on; 
ylabel('jointAngles')
leg = legend('jLeftAnkle','jLeftHip','Location','southeast');
                    set(leg,'Interpreter','latex');
                    set(leg,'FontSize',13);
 
xlabel('frames')

figure
subplot (211)
plot (P.tau(21,:));
hold on
plot (P.tau(15,:));
    axis tight;
    grid on;  
ylabel('jointTorques')
leg = legend('jRightAnkle','jRightHip','Location','southeast');
                    set(leg,'Interpreter','latex');
                    set(leg,'FontSize',13);
title ('Rotation on y axis');                   

subplot(212)
plot(human_state.q(23,:)*180/pi)
hold on
plot(human_state.q(5,:)*180/pi)
axis tight;
grid on; 
ylabel('jointAngles')
leg = legend('jRightAnkle','jRightHip','Location','southeast');
                    set(leg,'Interpreter','latex');
                    set(leg,'FontSize',13);
xlabel('frames')





