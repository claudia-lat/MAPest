
%% Preliminaries
close all;
powerTest = false;
CoC_analysisPlot = false;

for sjIdx = 1 : size(selectedJoints,1)
    % Right shoulder
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_rotx'))
        jRshoRotx_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_roty'))
        jRshoRoty_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_rotz'))
        jRshoRotz_idx = sjIdx;
    end
    % Left Shoulder
    if (strcmp(selectedJoints{sjIdx,1},'jLeftShoulder_rotx'))
        jLshoRotx_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jLeftShoulder_roty'))
        jLshoRoty_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jLeftShoulder_rotz'))
        jLshoRotz_idx = sjIdx;
    end
%     % C7shoulders
%     if (strcmp(selectedJoints{sjIdx,1},'jRightC7Shoulder_rotx'))
%         jRshoC7Rotx_idx = sjIdx;
%     end
%     if (strcmp(selectedJoints{sjIdx,1},'jLeftC7Shoulder_rotx'))
%         jLshoC7Rotx_idx = sjIdx;
%     end
end

%% Change of coordinates
% This step was useful to match the torques given by the Exo.
% To be consistent, the change has been done also for the trials with NO
% Exo.

for blockIdx = 1 : block.nrOfBlocks
    len = size(synchroKin(blockIdx).masterTime ,2);
    
    %% -------Right shoulder angles/torques analysis
    % Original angles from Opensim IK (q) in deg
    qx_rightSho = synchroKin(blockIdx).q(jRshoRotx_idx,:) * 180/pi; %deg
    qy_rightSho = synchroKin(blockIdx).q(jRshoRoty_idx,:) * 180/pi; %deg
    qz_rightSho = synchroKin(blockIdx).q(jRshoRotz_idx,:) * 180/pi; %deg
%     qx_rightC7Sho = synchroKin(blockIdx).q(jRshoC7Rotx_idx,:) * 180/pi; %deg
    
    % Torques estimated by MAP with angles q
    tau_rightSho = [estimatedVariables.tau(blockIdx).values(jRshoRotx_idx,:); ...
        estimatedVariables.tau(blockIdx).values(jRshoRoty_idx,:); ...
        estimatedVariables.tau(blockIdx).values(jRshoRotz_idx,:)];
%     tau_rightC7Sho = estimatedVariables.tau(blockIdx).values(jRshoC7Rotx_idx,:);
    
    if CoC_analysisPlot
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        
        subplot (421) % angles q
        plot(qx_rightSho,'r','Linewidth',1.5)
        hold on
        plot(qy_rightSho,'g','Linewidth',1.5)
        hold on
        plot(qz_rightSho,'b','Linewidth',1.5)
    %     hold on
    %     plot(qx_rightC7Sho,'m','Linewidth',1.5)
    %     hold on
    %     plot(qx_rightC7Sho + qx_rightSho,'k','Linewidth',1.5)
        ylabel('$q$ [deg]','FontSize',15,'Interpreter','latex');
        xlabel('samples','FontSize',15);
        title(sprintf('jRightShoulder, Block %s', num2str(blockIdx)))
        leg = legend('$q_x$','$q_y$','$q_z$'); %    leg = legend('$q_x$','$q_y$','$q_z$','$qC7_x$','$qTot_x$');
        set(leg,'FontSize',17)
        set(leg,'Interpreter','latex');

        subplot (423) %torques tau
        plot(tau_rightSho(1,:),'r','Linewidth',1.5)
        hold on
        plot(tau_rightSho(2,:),'g','Linewidth',1.5)
        hold on
        plot(tau_rightSho(3,:),'b','Linewidth',1.5)
    %     hold on
    %     plot(tau_rightC7Sho,'m','Linewidth',1.5)
        ylabel('$\tau$ [Nm]','FontSize',15,'Interpreter','latex');
        xlabel('samples','FontSize',15);
        leg = legend('$\tau_x$','$\tau_y$','$\tau_z$'); %leg = legend('$\tau_x$','$\tau_y$','$\tau_z$','${\tau}C7_x$');
        set(leg,'FontSize',17)
        set(leg,'Interpreter','latex');
    end
    
    qFirst_rightSho = zeros(3,len);
    jacobian_q_rightSho = cell(len,1);
    tauFirst_rightSho = zeros(3,len);
    for i = 1 : len
        % They are expressed in current frame (terna mobile) and the order of
        % consecutive rotations is: R_q = R_qx * R_qy * R_qz (from models)
        q_rightSho = [synchroKin(blockIdx).q(jRshoRotx_idx,i); ...
            synchroKin(blockIdx).q(jRshoRoty_idx,i); ...
            synchroKin(blockIdx).q(jRshoRotz_idx,i)];
        [R_qx_rightSho, R_qy_rightSho, R_qz_rightSho] = angle2rots(q_rightSho);
        R_q_rightSho = R_qx_rightSho * R_qy_rightSho * R_qz_rightSho;
        
        % Compute angles(qFirst) with the coordinates change
        qxFirst_rightSho = atan2(R_q_rightSho(3,2),sqrt(R_q_rightSho(1,2)^2 + R_q_rightSho(2,2)^2));
        qyFirst_rightSho = atan2(-R_q_rightSho(3,1),R_q_rightSho(3,3));
        qzFirst_rightSho = atan2(-R_q_rightSho(1,2),R_q_rightSho(2,2));
        qFirst_rightSho(:,i) = [qxFirst_rightSho; qyFirst_rightSho; qzFirst_rightSho] * 180/pi; %deg;
        
        % Compute the Jacobian J_q
        % considering equations (417),(291) in
        % 'Representing Attitude: Euler Angles, Unit Quaternions and Rotation Vectors', James Diebel, Staford 2006.
        xR = q_rightSho(1);%rad
        yR = q_rightSho(2);%rad
        zR = q_rightSho(3);%rad
        
        % the following relation comes from:
        % E_{xyz}(q)*qDot = E_{zxy}(qFirst)*qDotFirst
        % thus
        % J(q) = inv(E_{zxy}(qFirst)) * (E_{xyz}(q)*qDot)

        rS11 = -sin(yR)*cos(zR)/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2) + sin(zR)*cos(yR)*cos(zR)/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2);
        rS12 = sin(zR)^2/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2);
        rS13 = -cos(zR)/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2);

        rS21 = -sin(yR)*sin(zR)*cos(yR)/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2) - cos(yR)^2*cos(zR)^2/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2);
        rS22 = -sin(zR)*cos(yR)*cos(zR)/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2);
        rS23 = -sin(zR)*cos(yR)/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2);

        rS31 = -sin(zR)*cos(yR) + sin(yR)^2*cos(zR)/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2) - sin(yR)*sin(zR)*cos(yR)*cos(zR)/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2);
        rS32 = cos(zR) - sin(yR)*sin(zR)^2/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2);
        rS33 = sin(yR)*cos(zR)/(-sin(zR)^2*cos(yR) - cos(yR)*cos(zR)^2);

        jacobian_q_rightSho{i} = [rS11, rS12, rS13;
                                  rS21, rS22, rS23;
                                  rS31, rS32, rS33];

        % Compute the new torque (tauFirst) associated to the angle qFirst
        % (procedure obtained by applying D'Alambert principle)
        tauFirst_rightSho(:,i) = inv(jacobian_q_rightSho{i})' * tau_rightSho(:,i);
    end
    
    if CoC_analysisPlot
        subplot (425) %angles qFirst
        plot(qFirst_rightSho(1,:),'r','Linewidth',1.5)
        hold on
        plot(qFirst_rightSho(2,:),'g','Linewidth',1.5)
        hold on
        plot(qFirst_rightSho(3,:),'b','Linewidth',1.5)
        ylabel('$q\prime$ [deg]','FontSize',15,'Interpreter','latex');
        xlabel('samples','FontSize',15);
    %     title(sprintf('jRightShoulder, Block %s', num2str(blockIdx)))
        leg = legend('${q_x}\prime$','${q_y}\prime$','${q_z}\prime$');
        set(leg,'FontSize',17)
        set(leg,'Interpreter','latex');

        subplot (427) %torques tauFirst
        plot(tauFirst_rightSho(1,:),'r','Linewidth',1.5)
        hold on
        plot(tauFirst_rightSho(2,:),'g','Linewidth',1.5)
        hold on
        plot(tauFirst_rightSho(3,:),'b','Linewidth',1.5)
        ylabel('$\tau\prime$ [Nm]','FontSize',15,'Interpreter','latex');
        xlabel('samples','FontSize',15);
        leg = legend('${\tau_x}\prime$','${\tau_y}\prime$','${\tau_z}\prime$');
        set(leg,'FontSize',17)
        set(leg,'Interpreter','latex');
    end
    %% -------Left shoulder angles/torques analysis
    
    % Original angles from Opensim IK (q) in deg
    qx_leftSho = synchroKin(blockIdx).q(jLshoRotx_idx,:) * 180/pi; %deg
    qy_leftSho = synchroKin(blockIdx).q(jLshoRoty_idx,:) * 180/pi; %deg
    qz_leftSho = synchroKin(blockIdx).q(jLshoRotz_idx,:) * 180/pi; %deg
%     qx_leftC7Sho = synchroKin(blockIdx).q(jLshoC7Rotx_idx,:) * 180/pi; %deg
    
    % Torques estimated by MAP with angles q
    tau_leftSho = [estimatedVariables.tau(blockIdx).values(jLshoRotx_idx,:); ...
        estimatedVariables.tau(blockIdx).values(jLshoRoty_idx,:); ...
        estimatedVariables.tau(blockIdx).values(jLshoRotz_idx,:)];
%     tau_leftC7Sho = estimatedVariables.tau(blockIdx).values(jLshoC7Rotx_idx,:);
    
    if CoC_analysisPlot
        subplot (422) % angles q
        plot(qx_leftSho,'r','Linewidth',1.5)
        hold on
        plot(qy_leftSho,'g','Linewidth',1.5)
        hold on
        plot(qz_leftSho,'b','Linewidth',1.5)
    %     hold on
    %     plot(qx_leftC7Sho,'m','Linewidth',1.5)
    %     hold on
    %     plot(qx_leftC7Sho + qx_leftSho,'k','Linewidth',1.5)
        ylabel('$q$ [deg]','FontSize',15,'Interpreter','latex');
        xlabel('samples','FontSize',15);
        title(sprintf('jLeftShoulder, Block %s', num2str(blockIdx)))
        leg = legend('$q_x$','$q_y$','$q_z$'); %leg = legend('$q_x$','$q_y$','$q_z$','$qC7_x$', '$qTot_x$');
        set(leg,'FontSize',17)
        set(leg,'Interpreter','latex');

        subplot (424) %torques tau
        plot(tau_leftSho(1,:),'r','Linewidth',1.5)
        hold on
        plot(tau_leftSho(2,:),'g','Linewidth',1.5)
        hold on
        plot(tau_leftSho(3,:),'b','Linewidth',1.5)
    %     hold on
    %     plot(tau_leftC7Sho,'m','Linewidth',1.5)
        ylabel('$\tau$ [Nm]','FontSize',15,'Interpreter','latex');
        xlabel('samples','FontSize',15);
        leg = legend('$\tau_x$','$\tau_y$','$\tau_z$'); %leg = legend('$\tau_x$','$\tau_y$','$\tau_z$','${\tau}C7_x$');
        set(leg,'FontSize',17)
        set(leg,'Interpreter','latex');
     end
 
    qFirst_leftSho = zeros(3,len);
    jacobian_q_leftSho = cell(len,1);
    tauFirst_leftSho = zeros(3,len);
    for i = 1 : len
        % They are expressed in current frame (terna mobile) and the order of
        % consecutive rotations is: R_q = R_qx * R_qy * R_qz (from models)
        q_leftSho = [synchroKin(blockIdx).q(jLshoRotx_idx,i); ...
            synchroKin(blockIdx).q(jLshoRoty_idx,i); ...
            synchroKin(blockIdx).q(jLshoRotz_idx,i)];
        [R_qx_leftSho, R_qy_leftSho, R_qz_leftSho] = angle2rots(q_leftSho);
        R_q_leftSho = R_qx_leftSho * R_qy_leftSho * R_qz_leftSho;
        
        % Compute angles(qFirst) with the coordinates change
        qxFirst_leftSho = atan2(R_q_leftSho(3,2),sqrt(R_q_leftSho(1,2)^2 + R_q_leftSho(2,2)^2));
        qyFirst_leftSho = atan2(-R_q_leftSho(3,1),R_q_leftSho(3,3));
        qzFirst_leftSho = atan2(-R_q_leftSho(1,2),R_q_leftSho(2,2));
        qFirst_leftSho(:,i) = [qxFirst_leftSho; qyFirst_leftSho; qzFirst_leftSho] * 180/pi; %deg;

        % Compute the Jacobian J_q
        % considering equations (417),(291) in
        % 'Representing Attitude: Euler Angles, Unit Quaternions and Rotation Vectors', James Diebel, Staford 2006.
        xL = q_leftSho(1);%rad
        yL = q_leftSho(2);%rad
        zL = q_leftSho(3);%rad

        % the following relation comes from:
        % E_{xyz}(q)*qDot = E_{zxy}(qFirst)*qDotFirst
        % thus
        % J(q) = inv(E_{zxy}(qFirst)) * (E_{xyz}(q)*qDot)
        
        lS11 = -sin(yL)*cos(zL)/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2) + sin(zL)*cos(yL)*cos(zL)/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2);
        lS12 = sin(zL)^2/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2);
        lS13 = -cos(zL)/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2);

        lS21 = -sin(yL)*sin(zL)*cos(yL)/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2) - cos(yL)^2*cos(zL)^2/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2);
        lS22 = -sin(zL)*cos(yL)*cos(zL)/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2);
        lS23 = -sin(zL)*cos(yL)/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2);

        lS31 = -sin(zL)*cos(yL) + sin(yL)^2*cos(zL)/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2) - sin(yL)*sin(zL)*cos(yL)*cos(zL)/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2);
        lS32 = cos(zL) - sin(yL)*sin(zL)^2/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2);
        lS33 = sin(yL)*cos(zL)/(-sin(zL)^2*cos(yL) - cos(yL)*cos(zL)^2);

        jacobian_q_leftSho{i} = [lS11, lS12, lS13;
                                 lS21, lS22, lS23;
                                 lS31, lS32, lS33];

        % Compute the new torque (tauFirst) associated to the angle qFirst
        % (procedure obtained by applying D'Alambert principle)
        tauFirst_leftSho(:,i) = inv(jacobian_q_leftSho{i})' * tau_leftSho(:,i);

    end
    
    if CoC_analysisPlot
        subplot (426) %angles qFirst
        plot(qFirst_leftSho(1,:),'r','Linewidth',1.5)
        hold on
        plot(qFirst_leftSho(2,:),'g','Linewidth',1.5)
        hold on
        plot(qFirst_leftSho(3,:),'b','Linewidth',1.5)
        ylabel('$q\prime$ [deg]','FontSize',15,'Interpreter','latex');
        xlabel('samples','FontSize',15);
    %     title(sprintf('jLeftShoulder, Block %s', num2str(blockIdx)))
        leg = legend('${q_x}\prime$','${q_y}\prime$','${q_z}\prime$');
        set(leg,'FontSize',17)
        set(leg,'Interpreter','latex');

        subplot (428) %torques tauFirst
        plot(tauFirst_leftSho(1,:),'r','Linewidth',1.5)
        hold on
        plot(tauFirst_leftSho(2,:),'g','Linewidth',1.5)
        hold on
        plot(tauFirst_leftSho(3,:),'b','Linewidth',1.5)
        ylabel('$\tau\prime$ [Nm]','FontSize',15,'Interpreter','latex');
        xlabel('samples','FontSize',15);
        leg = legend('${\tau_x}\prime$','${\tau_y}\prime$','${\tau_z}\prime$');
        set(leg,'FontSize',17)
        set(leg,'Interpreter','latex');
    end
    
    %% Save CoC data in a struct
    CoC(blockIdx).block = block.labels(blockIdx);
    CoC(blockIdx).masterTime = synchroKin(blockIdx).masterTime;
    % right shoulder
    CoC(blockIdx).Rsho_q =[qx_rightSho; qy_rightSho; qz_rightSho]; %deg
    CoC(blockIdx).Rsho_tau = tau_rightSho;
    CoC(blockIdx).J_right = jacobian_q_rightSho;
    CoC(blockIdx).Rsho_qFirst = qFirst_rightSho; %deg
    CoC(blockIdx).Rsho_tauFirst = tauFirst_rightSho;
    % left shoulder
    CoC(blockIdx).Lsho_q =[qx_leftSho; qy_leftSho; qz_leftSho]; %deg
    CoC(blockIdx).Lsho_tau = tau_leftSho;
    CoC(blockIdx).J_left = jacobian_q_leftSho;
    CoC(blockIdx).Lsho_qFirst = qFirst_leftSho; %deg
    CoC(blockIdx).Lsho_tauFirst = tauFirst_leftSho;

end

%% Power test
% test if qDot'*tau = qDotFirst'*tauFirst
% LHS = qDot'*tau
% RHS = qDotFirst'*tauFirst = (qDot'*J') *tauFirst
if powerTest
    clearvars powerTest;

    for blockIdx = 1 : block.nrOfBlocks
        len = size(synchroKin(blockIdx).masterTime ,2);
        
        % right
        qDot_rightSho = [synchroKin(blockIdx).dq(jRshoRotx_idx,:); ...
            synchroKin(blockIdx).dq(jRshoRoty_idx,:); ...
            synchroKin(blockIdx).dq(jRshoRotz_idx,:)]; % rad
        %left
        qDot_leftSho = [synchroKin(blockIdx).dq(jLshoRotx_idx,:); ...
            synchroKin(blockIdx).dq(jLshoRoty_idx,:); ...
            synchroKin(blockIdx).dq(jLshoRotz_idx,:)]; % rad

        LHS_right_tmp = zeros(len,1);
        RHS_right_tmp = zeros(len,1);
        LHS_left_tmp  = zeros(len,1);
        RHS_left_tmp  = zeros(len,1);

        for i = 1 : len
            %rightSho
            LHS_right_tmp(i,1) = (qDot_rightSho(:,i))' * (CoC(blockIdx).Rsho_tau(:,i));
            RHS_right_tmp(i,1) = (qDot_rightSho(:,i))' * CoC(blockIdx).J_right{i, 1}'*(CoC(blockIdx).Rsho_tauFirst(:,i));
            %leftSho
            LHS_left_tmp(i,1)  = (qDot_leftSho(:,i))' * (CoC(blockIdx).Lsho_tau(:,i));
            RHS_left_tmp(i,1)  = (qDot_leftSho(:,i))' * CoC(blockIdx).J_left{i, 1}'*(CoC(blockIdx).Lsho_tauFirst(:,i));
        end
        powerTest(blockIdx).block = block.labels(blockIdx);
        powerTest(blockIdx).LHS_right = LHS_right_tmp;
        powerTest(blockIdx).RHS_right = RHS_right_tmp;
        powerTest(blockIdx).LHS_left  = LHS_left_tmp;
        powerTest(blockIdx).RHS_left  = RHS_left_tmp;

        powerTest(blockIdx).diffRigth = LHS_right_tmp - RHS_right_tmp;
        powerTest(blockIdx).diffLeft  = LHS_left_tmp - RHS_left_tmp;
    end
end



%% Utility
function [Rx, Ry, Rz] = angle2rots(x)
%ANGLE2ROTS computes the three rotation matrices given a vector x of angles
% 3x1 expressed in radians.
% It is required that the vector x is ordered as follow:
%    | angle of the rotation around x |
% x =| angle of the rotation around y |
%    | angle of the rotation around z |

Rx = [ 1     0          0    ;
       0 cos(x(1)) -sin(x(1));
       0 sin(x(1))  cos(x(1))];

Ry = [ cos(x(2)) 0 sin(x(2));
           0     1     0    ;
      -sin(x(2)) 0 cos(x(2))];

Rz = [ cos(x(3)) -sin(x(3)) 0;
       sin(x(3))  cos(x(3)) 0;
           0          0     1];
end



