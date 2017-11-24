%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  THIS SCRIPT IS FOR THE SENSOR COMBINATION ftShoes + Forceplates  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The script allows to validate shoes and forceplates.  The suit struct is
% not required for this comparison.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%The signals situation is the following:
% SUIT         |-----|------------------|-------|   240Hz, unix xsens time
% FP Upsampled |-----|------------------|-------|   240Hz, abs cortex time
% SHOES              |------------------|           100Hz, unix yarp time


%% Cut forceplates with suit rangeCut
comparison.forceplates.cut.time = comparison.forceplates.upsampled.time(comparison.rangeCut);
comparison.forceplates.cut.FP1.wrenches = comparison.forceplates.upsampled.FP1.wrenches(comparison.rangeCut,:);
comparison.forceplates.cut.FP2.wrenches = comparison.forceplates.upsampled.FP2.wrenches(comparison.rangeCut,:);
% we know at the current stage that suit and forceplates are synchro!

%% Downsample forceplates to shoes

% transform rel time into abs time
shoes_time_abs = zeros(size(comparison.shoes.time));
for i = 1 : size(comparison.shoes.time,2)
    shoes_time_abs(:,i) = comparison.shoes.time(:,i) - comparison.shoes.time(:,1);
end

fp_time_abs = zeros(size(comparison.forceplates.cut.time));
for i = 1 : size(comparison.forceplates.cut.time,2)
    fp_time_abs(:,i) = comparison.forceplates.cut.time(:,i) - comparison.forceplates.cut.time(:,1);
end                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                

comparison.masterTime = shoes_time_abs; % abs time
comparison.slaveTime = fp_time_abs; % abs time

for i = 1 : size(comparison.forceplates.cut.FP1.wrenches,2)
    comparison.forceplates.downsampled.FP1.wrenches(:,i) = interp1(comparison.slaveTime, ...
                                         comparison.forceplates.cut.FP1.wrenches(:,i), ...
                                         comparison.masterTime);
    comparison.forceplates.downsampled.FP2.wrenches(:,i) = interp1(comparison.slaveTime, ...
                                         comparison.forceplates.cut.FP2.wrenches(:,i), ...
                                         comparison.masterTime);
end

%% Plot raw comparison (no common ref. frames!)
% plotting raw z-axis forces from fp and shoes. At this stage they are not 
% in a common reference frame --> they have to be transformed into the
% human frames for both feet.

fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;

subplot (211) % only z axis FP1-Left
plot1 = plot(comparison.forceplates.downsampled.FP1.wrenches(:,3),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Left.forces(3,:),'r','lineWidth',1.5);
ylabel('FP1-Left','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0  size(comparison.shoes.Left.forces(3,:),2)])
grid on;

subplot (212) %only z axis FP2-Right
plot1 = plot(comparison.forceplates.downsampled.FP2.wrenches(:,3),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Right.forces(3,:),'r','lineWidth',1.5);
ylabel('FP2-Right','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0  size(comparison.shoes.Left.forces(3,:),2)])   
grid on;

leg = legend([plot1,plot2],{'fp','shoe'});
set(leg,'Interpreter','latex', ...
       'Position',[0.369020817175207 0.95613614004149 0.303215550427647 0.0305007585806261], ...
       'Orientation','horizontal');
set(leg,'FontSize',13);

%% Transform shoes into human frames
comparison.gravityZero = iDynTree.Vector3();
comparison.gravityZero.zero();

% LEFT -------------------------------------------------------------
comparison.leftHeel_T_leftFtShoeRot = iDynTree.Rotation();
comparison.leftHeel_T_leftFtShoeRot.fromMatlab([ 1.0,  0.0,  0.0; ...
                                                 0.0,  1.0,  0.0; ...
                                                 0.0,  0.0,  1.0]);
comparison.leftHeel_T_leftFtShoePos = iDynTree.Position();
comparison.leftFtShoeSeenFromLeftHeel = [0.037; 0 ; -0.050];
comparison.leftHeel_T_leftFtShoePos.fromMatlab(comparison.leftFtShoeSeenFromLeftHeel); % in m
comparison.leftFoot_T_leftHeelPos = iDynTree.Position();
comparison.leftHeelSeenFromLeftFoot = subjectParamsFromData.pLeftHeelFoot;
comparison.leftFoot_T_leftHeelPos.fromMatlab(comparison.leftHeelSeenFromLeftFoot); % in m
comparison.leftFoot_T_leftFtShoe = iDynTree.Transform(comparison.leftHeel_T_leftFtShoeRot,...
                        comparison.leftFoot_T_leftHeelPos + comparison.leftHeel_T_leftFtShoePos);

% RIGHT -------------------------------------------------------------
comparison.rightHeel_T_rightFtShoeRot = iDynTree.Rotation();
comparison.rightHeel_T_rightFtShoeRot.fromMatlab([ 1.0,  0.0,  0.0; ...
                                                   0.0,  1.0,  0.0; ...
                                                   0.0,  0.0,  1.0]);
comparison.rightHeel_T_rightFtShoePos = iDynTree.Position();
comparison.rightFtShoeSeenFromRightHeel = [0.037; 0 ; -0.050];
comparison.rightHeel_T_rightFtShoePos.fromMatlab(comparison.rightFtShoeSeenFromRightHeel); % in m
comparison.rightFoot_T_rightHeelPos = iDynTree.Position();
comparison.rightHeelSeenFromRightFoot = subjectParamsFromData.pRightHeelFoot;
comparison.rightFoot_T_rightHeelPos.fromMatlab(comparison.rightHeelSeenFromRightFoot); % in m
comparison.rightFoot_T_rightFtShoe = iDynTree.Transform(comparison.rightHeel_T_rightFtShoeRot,...
                          comparison.rightFoot_T_rightHeelPos + comparison.rightHeel_T_rightFtShoePos);

comparison.leftShoeWrench(1:3,:) = comparison.shoes.Left.forces;
comparison.leftShoeWrench(4:6,:) = comparison.shoes.Left.moments;
comparison.rightShoeWrench(1:3,:) = comparison.shoes.Right.forces;
comparison.rightShoeWrench(4:6,:) = comparison.shoes.Right.moments;

comparison.shoes.Left.humanFootWrench = ...
      -1*(comparison.leftFoot_T_leftFtShoe.asAdjointTransformWrench().toMatlab()*(comparison.leftShoeWrench));
comparison.shoes.Right.humanFootWrench = ...
      -1*(comparison.rightFoot_T_rightFtShoe.asAdjointTransformWrench().toMatlab()*(comparison.rightShoeWrench));

%% Transform forceplates into human frames 
comparison.shoeHeight = 0.060; % in m

% Extract wrenches and position from forceplate data 
comparison.fp1Wrench = comparison.forceplates.downsampled.FP1.wrenches';
comparison.fp2Wrench = comparison.forceplates.downsampled.FP2.wrenches';

% Useful information for these transformation are in:
% - rawSketch.jpg  --> for the rotation between each foot wrt the related 
%                      forceplate;
% - fixtureUW.pdf  --> for the position of the each foot wrt the related
%                      forceplate taht is located at the center of the rear
%                      sensor (assumption: on this point there is a
%                      reference frame oriented as in the foot).

% ---- FP1 --> transform FP1 data from FP1 frame to leftFoot frame
% We want: leftFoot_f_fp1 starting from fp1_f_fp1
comparison.leftSole_T_fp1Pos = iDynTree.Position();
comparison.fp1SeenFromLeftSole = [0.099; 0.063 ; 0];
comparison.leftSole_T_fp1Pos.fromMatlab(comparison.fp1SeenFromLeftSole);
comparison.leftFoot_T_leftSolePos = iDynTree.Position();
comparison.leftSoleSeenFromLeftFoot = [0.0; 0.0; ...
                    subjectParamsFromData.leftFootBoxOrigin(3) - comparison.shoeHeight];
comparison.leftFoot_T_leftSolePos.fromMatlab(comparison.leftSoleSeenFromLeftFoot);
comparison.leftSole_R_fp1 = iDynTree.Rotation(); % ==leftFoot_R_fp1
comparison.leftSole_R_fp1.fromMatlab ([ 0.0, -1.0,  0.0; ...
                                       -1.0,  0.0,  0.0; ...
                                        0.0,  0.0, -1.0]);
comparison.leftFoot_T_fp1 = iDynTree.Transform(comparison.leftSole_R_fp1, ...
             comparison.leftFoot_T_leftSolePos + comparison.leftSole_T_fp1Pos);

% transform the wrench in the proper frame and change the sign
comparison.forceplates.FP1.humanLeftFootWrench = ...
              -1*(comparison.leftFoot_T_fp1.asAdjointTransformWrench().toMatlab()* ...
              comparison.fp1Wrench);

% ---- FP2 --> transform FP2 data from FP2 frame to rightFoot frame
% We want: rightFoot_f_fp2 starting from fp2_f_fp2
comparison.rightSole_T_fp2Pos = iDynTree.Position();
comparison.fp2SeenFromRightSole = [0.099; -0.064 ; 0];
comparison.rightSole_T_fp2Pos.fromMatlab(comparison.fp2SeenFromRightSole);
comparison.rightFoot_T_rightSolePos = iDynTree.Position();
comparison.rightSoleSeenFromRightFoot = [0.0; 0.0; ...
                    subjectParamsFromData.rightFootBoxOrigin(3) - comparison.shoeHeight];
comparison.rightFoot_T_rightSolePos.fromMatlab(comparison.rightSoleSeenFromRightFoot);
comparison.rightSole_R_fp2 = iDynTree.Rotation(); % ==rightFoot_R_fp2
comparison.rightSole_R_fp2.fromMatlab ([ 0.0, -1.0,  0.0; ...
                                        -1.0,  0.0,  0.0; ...
                                         0.0,  0.0, -1.0]);
comparison.rightFoot_T_fp2 = iDynTree.Transform(comparison.rightSole_R_fp2, ...
           comparison.rightFoot_T_rightSolePos + comparison.rightSole_T_fp2Pos);

% transform the wrench in the proper frame and change the sign
comparison.forceplates.FP2.humanRightFootWrench = ...
              -1*(comparison.rightFoot_T_fp2.asAdjointTransformWrench().toMatlab()* ...
              comparison.fp2Wrench);  
  
%% Plot comparison in human frames

%FP1-LEFT SHOE COMPARISON
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;
len = length(comparison.forceplates.FP1.humanLeftFootWrench);
              
subplot (231) % comparison forces component x
plot1 = plot(comparison.forceplates.FP1.humanLeftFootWrench(1,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Left.humanFootWrench(1,:),'r','lineWidth',1.5);
ylabel('forces','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0  len]);
title ('x');
grid on;


subplot (232) % comparison forces component y 
plot1 = plot(comparison.forceplates.FP1.humanLeftFootWrench(2,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Left.humanFootWrench(2,:),'r','lineWidth',1.5);
% xlim([0  length(comparison.forceplates.FP1.humanLeftFootWrench)]);
title ('y');
xlim([0  len]);
grid on;

subplot (233) % comparison forces component z
plot1 = plot(comparison.forceplates.FP1.humanLeftFootWrench(3,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Left.humanFootWrench(3,:),'r','lineWidth',1.5);
% xlim([0  length(comparison.forceplates.FP1.humanLeftFootWrench)]);
title ('z');
xlim([0  len]);
grid on;

subplot (234) % comparison moment component x 
plot1 = plot(comparison.forceplates.FP1.humanLeftFootWrench(4,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Left.humanFootWrench(4,:),'r','lineWidth',1.5);
ylabel('moments','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0  len]);
grid on;

subplot (235) % comparison moment component y
plot1 = plot(comparison.forceplates.FP1.humanLeftFootWrench(5,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Left.humanFootWrench(5,:),'r','lineWidth',1.5);
xlim([0  len]);
grid on;

subplot (236) % comparison moment component z 
plot1 = plot(comparison.forceplates.FP1.humanLeftFootWrench(6,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Left.humanFootWrench(6,:),'r','lineWidth',1.5);
xlim([0  len]);
grid on;

leg = legend([plot1,plot2],{'fp1','Left_shoe'});
set(leg,'Interpreter','latex', ...
       'Position',[0.369020817175207 0.95613614004149 0.303215550427647 0.0305007585806261], ...
       'Orientation','horizontal');
set(leg,'FontSize',13);


%FP2-RIGH SHOE COMPARISON
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;
              
subplot (231) % comparison forces component x
plot1 = plot(comparison.forceplates.FP2.humanRightFootWrench(1,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Right.humanFootWrench(1,:),'r','lineWidth',1.5);
ylabel('forces','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0  len]);
title ('x');
grid on;


subplot (232) % comparison forces component y 
plot1 = plot(comparison.forceplates.FP2.humanRightFootWrench(2,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Right.humanFootWrench(2,:),'r','lineWidth',1.5);
% xlim([0  length(comparison.forceplates.FP1.humanLeftFootWrench)]);
title ('y');
xlim([0  len]);
grid on;

subplot (233) % comparison forces component z
plot1 = plot(comparison.forceplates.FP2.humanRightFootWrench(3,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Right.humanFootWrench(3,:),'r','lineWidth',1.5);
% xlim([0  length(comparison.forceplates.FP1.humanLeftFootWrench)]);
title ('z');
xlim([0  len]);
grid on;

subplot (234) % comparison moment component x 
plot1 = plot(comparison.forceplates.FP2.humanRightFootWrench(4,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Right.humanFootWrench(4,:),'r','lineWidth',1.5);
ylabel('moments','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0  len]);
grid on;

subplot (235) % comparison moment component y
plot1 = plot(comparison.forceplates.FP2.humanRightFootWrench(5,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Right.humanFootWrench(5,:),'r','lineWidth',1.5);
xlim([0  len]);
grid on;

subplot (236) % comparison moment component z 
plot1 = plot(comparison.forceplates.FP2.humanRightFootWrench(6,:),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Right.humanFootWrench(6,:),'r','lineWidth',1.5);
xlim([0  len]);
grid on;

leg = legend([plot1,plot2],{'fp2','Right-shoe'});
set(leg,'Interpreter','latex', ...
       'Position',[0.369020817175207 0.95613614004149 0.303215550427647 0.0305007585806261], ...
       'Orientation','horizontal');
set(leg,'FontSize',13);
