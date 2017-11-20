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
% human frames for both foot.

fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
              box(axes1,'on');
              hold(axes1,'on');
              grid on;

subplot (211) % only z axis FP1-Left
plot1 = plot(-comparison.forceplates.downsampled.FP1.wrenches(:,3),'b','lineWidth',1.5);
hold on 
plot2 = plot(comparison.shoes.Left.forces(3,:),'r','lineWidth',1.5);
ylabel('FP1-Left','HorizontalAlignment','center',...
       'FontWeight','bold',...
       'FontSize',18,...
       'Interpreter','latex');
xlim([0  size(comparison.shoes.Left.forces(3,:),2)])
grid on;

subplot (212) %only z axis FP2-Right
plot1 = plot(-comparison.forceplates.downsampled.FP2.wrenches(:,3),'b','lineWidth',1.5);
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
