
% Path to the HDE folder data
bucket.pathTo_dHDE = fullfile(bucket.pathToTask,'data/dynamicVariablesVector.txt');
bucket.pathTo_yHDE = fullfile(bucket.pathToTask,'data/measurementsVector.txt');

bucket.pathTo_dHDEorder = fullfile(bucket.pathToTask,'data/dVectorOrder_HDE.txt');
bucket.pathTo_yHDEorder = fullfile(bucket.pathToTask,'data/yVectorOrder_HDE.txt');

dHDE_order_links = {'Pelvis';
    'LeftUpperLeg_f1';
    'L5_f1';
    'RightUpperLeg_f1';
    'RightUpperLeg_f2';
    'RightUpperLeg';
    'RightLowerLeg_f1';
    'RightLowerLeg';
    'RightFoot_f1';
    'RightFoot_f2';
    'RightFoot';
    'RightToe';
    'L5';
    'L3_f1';
    'L3';
    'T12_f1';
    'T12';
    'T8_f1';
    'T8_f2';
    'T8';
    'Neck_f1';
    'RightShoulder';
    'LeftShoulder';
    'LeftUpperArm_f1';
    'LeftUpperArm_f2';
    'LeftUpperArm';
    'LeftForeArm_f1';
    'LeftForeArm';
    'LeftHand_f1';
    'LeftHand';
    'RightUpperArm_f1';
    'RightUpperArm_f2';
    'RightUpperArm';
    'RightForeArm_f1';
    'RightForeArm';
    'RightHand_f1';
    'RightHand';
    'Neck_f2';
    'Neck';
    'Head_f1';
    'Head';
    'LeftUpperLeg_f2';
    'LeftUpperLeg';
    'LeftLowerLeg_f1';
    'LeftLowerLeg';
    'LeftFoot_f1';
    'LeftFoot_f2';
    'LeftFoot';
    'LeftToe';};

dHDE_order_joints = {'jLeftHip_rotx';
    'jL5S1_rotx';
    'jRightHip_rotx';
    'jRightHip_roty';
    'jRightHip_rotz';
    'jRightKnee_roty';
    'jRightKnee_rotz';
    'jRightAnkle_rotx';
    'jRightAnkle_roty';
    'jRightAnkle_rotz';
    'jRightBallFoot_roty';
    'jL5S1_roty';
    'jL4L3_rotx';
    'jL4L3_roty';
    'jL1T12_rotx';
    'jL1T12_roty';
    'jT9T8_rotx';
    'jT9T8_roty';
    'jT9T8_rotz';
    'jT1C7_rotx';
    'jRightC7Shoulder_rotx';
    'jLeftC7Shoulder_rotx';
    'jLeftShoulder_rotx';
    'jLeftShoulder_roty';
    'jLeftShoulder_rotz';
    'jLeftElbow_roty';
    'jLeftElbow_rotz';
    'jLeftWrist_rotx';
    'jLeftWrist_rotz';
    'jRightShoulder_rotx';
    'jRightShoulder_roty';
    'jRightShoulder_rotz';
    'jRightElbow_roty';
    'jRightElbow_rotz';
    'jRightWrist_rotx';
    'jRightWrist_rotz';
    'jT1C7_roty';
    'jT1C7_rotz';
    'jC1Head_rotx';
    'jC1Head_roty';
    'jLeftHip_roty';
    'jLeftHip_rotz';
    'jLeftKnee_roty';
    'jLeftKnee_rotz';
    'jLeftAnkle_rotx';
    'jLeftAnkle_roty';
    'jLeftAnkle_rotz';
    'jLeftBallFoot_roty'};

%% Extract values from HDE file
disp('-------------------------------------------------------------------');
disp('[Start] mu_dgiveny extraction from HDE file...')
% values
tmp.dHDE = readtable(bucket.pathTo_dHDE);
dHDE = (table2array(tmp.dHDE(:, 1:end-1)))';
% order
% delimiterIn = ' ';
% dHDEorder = importdata(bucket.pathTo_dHDEorder,delimiterIn);
% dHDEorder = importdata(bucket.pathTo_dHDEorder);
% dHDEorder = (table2array(tmp.dHDEorder));
% dVectorOrderHDE = table2cell(importfile(bucket.pathTo_dHDEorder));
% dVectorOrderHDE_new.order = dHDE_order_links;
% dVectorOrderHDE_new.range.linAcc =
disp('[End] mu_dgiveny extraction from HDE file')

disp('-------------------------------------------------------------------');
disp('[Start] y extraction from HDE file...')
% values
tmp.yHDE = readtable(bucket.pathTo_yHDE);
yHDE = (table2array(tmp.yHDE(:, 1:end-1)))';
% yHDE(52:102,:) = []; %manually removing
%order
disp('[End] y extraction from HDE file')

%% Comparison y vector
% fig = figure('Name', 'y comparison','NumberTitle','off');
% axes1 = axes('Parent',fig,'FontSize',16);
% box(axes1,'on');
% hold(axes1,'on');
%
% plot1 = plot(yHDE(:,1),'lineWidth',1.5);
% hold on
% plot2 = plot(y(:,1),'lineWidth',1.5);
% title('y vector');
%
% leg = legend([plot1,plot2],{'HDE','MAPest'},'Location','northeast');
% set(leg,'Interpreter','latex')
% %     'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %     'Orientation','horizontal');
% set(leg,'FontSize',18);

% ---- lin acc, direct order
fig = figure('Name', 'lin acc in y - comparison','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

plot1 = plot(yHDE(1:51,10),'lineWidth',1.5);
hold on
plot2 = plot(y(1:51,10),'lineWidth',1.5);
title('lin acc in y vector');

leg = legend([plot1,plot2],{'HDE','MAPest'},'Location','northeast');
set(leg,'Interpreter','latex')
%     'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
%     'Orientation','horizontal');
set(leg,'FontSize',18);

% ---- ddq, direct order
fig = figure('Name', ' ddq in y - comparison','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

plot3 = plot(yHDE(52:99,10),'lineWidth',1.5);
hold on
plot4 = plot(y(52:99,10),'lineWidth',1.5);
title('ddq in y vector');

leg = legend([plot3,plot4],{'HDE','MAPest'},'Location','northeast');
set(leg,'Interpreter','latex')
%     'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
%     'Orientation','horizontal');
set(leg,'FontSize',18);

% ---- fext, different order
fig = figure('Name', 'y comparison','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

yHDE_temp_fext = zeros(294,1);
yHDE_temp_fext(235:240,1) = yHDE(382:387,10);
yHDE_temp_fext(283:288,1) = yHDE(160:165,10);
plot5 = plot(yHDE_temp_fext(:,1),'lineWidth',1.5);
hold on
plot6 = plot(y(100:end,1),'lineWidth',1.5);
title('fext in y vector');
%
leg = legend([plot5,plot6],{'HDE','MAPest'},'Location','northeast');
set(leg,'Interpreter','latex')
%     'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
%     'Orientation','horizontal');
set(leg,'FontSize',18);

%% Comparison d vector
% -----------------------------------------------------------------------%
%  LINEAR ACCELERATION
% -----------------------------------------------------------------------%

linAcc_fromMAPest.order  = estimatedVariables.Acc.label;
linAcc_fromMAPest.values = estimatedVariables.Acc.values;

linAcc_fromHDE.order  = dHDE_order_links;
linAcc_fromHDE.values  = zeros(length(dHDE_order_links),size(dHDE,2));
% extract linAcc from complete dHDE
for i = 1 : length(dHDE_order_links)
    linAcc_fromHDE.values(6*(i-1)+1:6*i,:) = dHDE(12*(i-1)+1:(12*i)-6,:);
end

% order linAcc_fromHDE as the linAcc_fromMAPest
linAcc_fromHDE.values_neworder = [];
for linAccMAPestIdx = 1 : length(dHDE_order_links)
    for linAccHDEIdx = 1 : length(dHDE_order_links)
        if strcmp(linAcc_fromMAPest.order{linAccMAPestIdx},linAcc_fromHDE.order{linAccHDEIdx})
            linAcc_fromHDE.values_neworder = [linAcc_fromHDE.values_neworder; ...
                linAcc_fromHDE.values(6*(linAccHDEIdx-1)+1:6*linAccHDEIdx,:)];
            break;
        end
    end
end

% ----------------
% fig = figure('Name', 'lin acc in d - comparison','NumberTitle','off');
% axes1 = axes('Parent',fig,'FontSize',16);
% box(axes1,'on');
% hold(axes1,'on');
%
% plot1 = plot(linAcc_fromHDE.values_neworder(:,100),'lineWidth',1.5);
% hold on
% plot2 = plot(linAcc_fromMAPest.values(:,100),'lineWidth',1.5);
% title('lin acc in d vector');
%
% leg = legend([plot1,plot2],{'HDE','MAPest'},'Location','northeast');
% set(leg,'Interpreter','latex')
% %     'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %     'Orientation','horizontal');
% set(leg,'FontSize',18);

% ----------------
fig = figure('Name', 'linAcc comparison','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');

for linAccMAPestIdx = 1 : length(dHDE_order_links)
    subplot (5,10,linAccMAPestIdx)
    % from HDE
    plot1 = plot(linAcc_fromHDE.values_neworder(6*(linAccMAPestIdx-1)+1,:),'r','lineWidth',1.5);
    hold on
    plot2 = plot(linAcc_fromHDE.values_neworder(6*(linAccMAPestIdx-1)+2,:),'g','lineWidth',1.5);
    plot3 = plot(linAcc_fromHDE.values_neworder(6*(linAccMAPestIdx-1)+3,:),'b','lineWidth',1.5);
    % from MAPest
    plot7  = plot(linAcc_fromMAPest.values(6*(linAccMAPestIdx-1)+1,:),'r--o','lineWidth',0.5);
    plot8  = plot(linAcc_fromMAPest.values(6*(linAccMAPestIdx-1)+2,:),'g--o','lineWidth',0.5);
    plot9  = plot(linAcc_fromMAPest.values(6*(linAccMAPestIdx-1)+3,:),'b--o','lineWidth',0.5);
    grid on;
    
    title(sprintf('%s',linAcc_fromMAPest.order{linAccMAPestIdx}));
    
    if linAccMAPestIdx == 1
        leg = legend([plot1,plot7],{'HDE','MAPest'},'Location','northeast');
        set(leg,'Interpreter','latex', ...)
            'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
            'Orientation','horizontal');
        set(leg,'FontSize',18);
    end
end

% -----------------------------------------------------------------------%
% EXTERNAL FORCES
% -----------------------------------------------------------------------%

fext_fromMAPest.order  = estimatedVariables.Fext.label;
fext_fromMAPest.values = estimatedVariables.Fext.values;

fext_fromHDE.order  = dHDE_order_links;
fext_fromHDE.values  = zeros(length(dHDE_order_links),size(dHDE,2));
% extract fext from complete dHDE
for i = 1 : length(dHDE_order_links)
    fext_fromHDE.values(6*(i-1)+1:6*i,:) = dHDE(12*(i-1)+7:(12*i),:);
end

% order fext_fromHDE as the fext_fromMAPest
fext_fromHDE.values_neworder = [];
for fextMAPestIdx = 1 : length(dHDE_order_links)
    for fextHDEIdx = 1 : length(dHDE_order_links)
        if strcmp(fext_fromMAPest.order{fextMAPestIdx},fext_fromHDE.order{fextHDEIdx})
            fext_fromHDE.values_neworder = [fext_fromHDE.values_neworder; ...
                fext_fromHDE.values(6*(fextHDEIdx-1)+1:6*fextHDEIdx,:)];
            break;
        end
    end
end

% ----------------
% fig = figure('Name', 'fext in d - comparison','NumberTitle','off');
% axes1 = axes('Parent',fig,'FontSize',16);
% box(axes1,'on');
% hold(axes1,'on');
%
% plot1 = plot(fext_fromHDE.values_neworder(:,1),'lineWidth',1.5);
% hold on
% plot2 = plot(fext_fromMAPest.values(:,1),'lineWidth',1.5);
% title('fext in d vector');
%
% leg = legend([plot1,plot2],{'HDE','MAPest'},'Location','northeast');
% set(leg,'Interpreter','latex')
% %     'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %     'Orientation','horizontal');
% set(leg,'FontSize',18);
% ----------------

fig = figure('Name', 'fext comparison','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');

for fextMAPestIdx = 1 : length(dHDE_order_links)
    subplot (5,10,fextMAPestIdx)
    % from HDE
    plot1 = plot(fext_fromHDE.values_neworder(6*(fextMAPestIdx-1)+1,:),'r','lineWidth',1.5);
    hold on
    plot2 = plot(fext_fromHDE.values_neworder(6*(fextMAPestIdx-1)+2,:),'g','lineWidth',1.5);
    plot3 = plot(fext_fromHDE.values_neworder(6*(fextMAPestIdx-1)+3,:),'b','lineWidth',1.5);
    % from MAPest
    plot7  = plot(fext_fromMAPest.values(6*(fextMAPestIdx-1)+1,:),'r--o','lineWidth',0.5);
    plot8  = plot(fext_fromMAPest.values(6*(fextMAPestIdx-1)+2,:),'g--o','lineWidth',0.5);
    plot9  = plot(fext_fromMAPest.values(6*(fextMAPestIdx-1)+3,:),'b--o','lineWidth',0.5);
    grid on;
    
    title(sprintf('%s',fext_fromMAPest.order{fextMAPestIdx}));
    
    if fextMAPestIdx ==1
        leg = legend([plot1,plot7],{'HDE','MAPest'},'Location','northeast');
        set(leg,'Interpreter','latex', ...)
            'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
            'Orientation','horizontal');
        set(leg,'FontSize',18);
    end
end

% -----------------------------------------------------------------------%
% INTERNAL FORCES
% -----------------------------------------------------------------------%

fint_fromMAPest.order  = estimatedVariables.Fint.label;
fint_fromMAPest.values = estimatedVariables.Fint.values;

fint_fromHDE.order  = dHDE_order_joints;
fint_fromHDE.values  = zeros(length(dHDE_order_joints),size(dHDE,2));
% extract fint from complete dHDE
for i = 1 : length(dHDE_order_joints)
    fint_fromHDE.values(6*(i-1)+1:6*i,:) = dHDE(6*(i-1)+589:(6*i + 588),:);
end

% order fint_fromHDE as the fint_fromMAPest
fint_fromHDE.values_neworder = [];
for fintMAPestIdx = 1 : length(dHDE_order_joints)
    for fintHDEIdx = 1 : length(dHDE_order_joints)
        if strcmp(fint_fromMAPest.order{fintMAPestIdx},fint_fromHDE.order{fintHDEIdx})
            fint_fromHDE.values_neworder = [fint_fromHDE.values_neworder; ...
                fint_fromHDE.values(6*(fintHDEIdx-1)+1:6*fintHDEIdx,:)];
            break;
        end
    end
end

fig = figure('Name', 'fint comparison','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');

for fintMAPestIdx = 1 : length(dHDE_order_joints)
    subplot (5,10,fintMAPestIdx)
    % from HDE
    plot1 = plot(fint_fromHDE.values_neworder(6*(fintMAPestIdx-1)+1,:),'r','lineWidth',1.5);
    hold on
    plot2 = plot(fint_fromHDE.values_neworder(6*(fintMAPestIdx-1)+2,:),'g','lineWidth',1.5);
    plot3 = plot(fint_fromHDE.values_neworder(6*(fintMAPestIdx-1)+3,:),'b','lineWidth',1.5);
    % from MAPest
    plot7  = plot(fint_fromMAPest.values(6*(fintMAPestIdx-1)+1,:),'r--o','lineWidth',0.5);
    plot8  = plot(fint_fromMAPest.values(6*(fintMAPestIdx-1)+2,:),'g--o','lineWidth',0.5);
    plot9  = plot(fint_fromMAPest.values(6*(fintMAPestIdx-1)+3,:),'b--o','lineWidth',0.5);
    grid on;
    
    title(sprintf('%s',fint_fromMAPest.order{fintMAPestIdx}));
    
    if fintMAPestIdx ==1
        leg = legend([plot1,plot7],{'HDE','MAPest'},'Location','northeast');
        set(leg,'Interpreter','latex', ...)
            'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
            'Orientation','horizontal');
        set(leg,'FontSize',18);
    end
end

% -----------------------------------------------------------------------%
% JOINT ACCELERATION
% -----------------------------------------------------------------------%
%%
ddq_fromMAPest.order  = estimatedVariables.ddq.label;
ddq_fromMAPest.values = estimatedVariables.ddq.values;

ddq_fromHDE.order  = dHDE_order_joints;
ddq_fromHDE.values = [];
% extract ddq from complete dHDE
for i = 1 : length(dHDE_order_joints)
    ddq_fromHDE.values = [ddq_fromHDE.values; dHDE((i-1)+872,:)];
end

% order ddq_fromHDE as the ddq_fromMAPest
ddq_fromHDE.values_neworder = [];
for ddqMAPestIdx = 1 : length(dHDE_order_joints)
    for ddqHDEIdx = 1 : length(dHDE_order_joints)
        if strcmp(ddq_fromMAPest.order{ddqMAPestIdx},ddq_fromHDE.order{ddqHDEIdx})
            ddq_fromHDE.values_neworder = [ddq_fromHDE.values_neworder; ...
                ddq_fromHDE.values(ddqHDEIdx,:)];
            break;
        end
    end
end

fig = figure('Name', 'ddq comparison','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');

for ddqMAPestIdx = 1 : length(dHDE_order_joints)
    subplot (5,10,ddqMAPestIdx)
    % from HDE
    plot1 = plot(ddq_fromHDE.values_neworder(ddqMAPestIdx,:),'k','lineWidth',2);
    hold on
    % from MAPest
    plot2  = plot(ddq_fromMAPest.values(ddqMAPestIdx,:),'m','lineWidth',2);
    grid on;
    
    title(sprintf('%s',ddq_fromMAPest.order{ddqMAPestIdx}));
    
    if ddqMAPestIdx == 1
        leg = legend([plot1,plot2],{'HDE','MAPest'},'Location','northeast');
        set(leg,'Interpreter','latex', ...)
            'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
            'Orientation','horizontal');
        set(leg,'FontSize',18);
    end
end

%% Clearvars
clearvars tmp;
