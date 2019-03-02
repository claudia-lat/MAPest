% Plot the estimated joint torques w.r.t. the contact pattern detected
% close all;

% Plot folder
bucket.pathToPlots = fullfile(bucket.pathToTask,'plots');
if ~exist(bucket.pathToPlots)
    mkdir (bucket.pathToPlots)
end
saveON = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% RIGHT ANKLE JOINT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for tauIdx = 1 : nrDofs
    % ----------------------- Right Ankle rotx ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightAnkle_rotx')
        torque_RA_rotx = estimatedVariables.tau.values(tauIdx,:);
        maxVal_RA_rotx = max(torque_RA_rotx);
        minVal_RA_rotx = min(torque_RA_rotx);
    end
    % ----------------------- Right Ankle roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightAnkle_roty')
        torque_RA_roty = estimatedVariables.tau.values(tauIdx,:);
        maxVal_RA_roty = max(torque_RA_roty);
        minVal_RA_roty = min(torque_RA_roty);
    end
    % ----------------------- Right Ankle roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightAnkle_rotz')
        torque_RA_rotz = estimatedVariables.tau.values(tauIdx,:);
        maxVal_RA_rotz = max(torque_RA_rotz);
        minVal_RA_rotz = min(torque_RA_rotz);
    end
end
%% Decide a ylim for all the rot
maxVal = max([maxVal_RA_rotx, maxVal_RA_roty, maxVal_RA_rotz]);
minVal = min([minVal_RA_rotx, minVal_RA_roty, minVal_RA_rotz]);

%% Plot
fig = figure('Name', 'Joint torque w.r.t. pattern detection','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

% Colors
doubleSupportColor       = [0.87058824300766 0.921568632125854 0.980392158031464];
singleSupportColor_left  = [1 0.800000011920929 0.800000011920929];
singleSupportColor_right = [0.756862759590149 0.866666674613953 0.776470601558685];

subplot (311) % Right Ankle rotx ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_RA_rotx,'k','lineWidth',1.5);

title(sprintf('Right Ankle Joint, Subj %s, Task %s',num2str(subjectID),num2str(taskID)));
ylabel('\tau_{x} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;


subplot (312) % Right Ankle roty ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_RA_roty,'k','lineWidth',1.5);

ylabel('\tau_{y} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;

subplot (313) % Right Ankle rotz ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_RA_rotz,'k','lineWidth',1.5);

ylabel('\tau_{z} [Nm]');
xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;

%legend
if flag_SSleft % DS + leftSS
    leg = legend([plot1,shadedPatch1,shadedPatch2], ...
        {'\tau_{MAP}','doubleSupport','leftSupport'},'Location','northeast','FontSize',18);
end
if flag_SSright % DS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','rightSupport'},'Location','northeast','FontSize',18);
end
if flag_SSleft & flag_SSright % DS + leftSS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch2,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','leftSupport','rightSupport'},'Location','northeast','FontSize',18);
end

subplotsqueeze(gcf, 1.2);
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('right_ankle')),fig,600);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LEFT ANKLE JOINT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for tauIdx = 1 : nrDofs
    % ----------------------- Left Ankle rotx ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftAnkle_rotx')
        torque_LA_rotx = estimatedVariables.tau.values(tauIdx,:);
        maxVal_LA_rotx = max(torque_LA_rotx);
        minVal_LA_rotx = min(torque_LA_rotx);
    end
    % ----------------------- Left Ankle roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftAnkle_roty')
        torque_LA_roty = estimatedVariables.tau.values(tauIdx,:);
        maxVal_LA_roty = max(torque_LA_roty);
        minVal_LA_roty = min(torque_LA_roty);
    end
    % ----------------------- Left Ankle roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftAnkle_rotz')
        torque_LA_rotz = estimatedVariables.tau.values(tauIdx,:);
        maxVal_LA_rotz = max(torque_LA_rotz);
        minVal_LA_rotz = min(torque_LA_rotz);
    end
end

%% Decide a ylim for all the rot
maxVal = max([maxVal_LA_rotx, maxVal_LA_roty, maxVal_LA_rotz]);
minVal = min([minVal_LA_rotx, minVal_LA_roty, minVal_LA_rotz]);

%% Plot
fig = figure('Name', 'Joint torque w.r.t. pattern detection','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

% Colors
doubleSupportColor       = [0.87058824300766 0.921568632125854 0.980392158031464];
singleSupportColor_left  = [1 0.800000011920929 0.800000011920929];
singleSupportColor_right = [0.756862759590149 0.866666674613953 0.776470601558685];

subplot (311) % Left Ankle rotx ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_LA_rotx,'k','lineWidth',1.5);

title(sprintf('Left Ankle Joint, Subj %s, Task %s',num2str(subjectID),num2str(taskID)));
ylabel('\tau_{x} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;


subplot (312) % Left Ankle roty ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_LA_roty,'k','lineWidth',1.5);

ylabel('\tau_{y} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;

subplot (313) % Left Ankle rotz ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_LA_rotz,'k','lineWidth',1.5);

ylabel('\tau_{z} [Nm]');
xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;

%legend
if flag_SSleft % DS + leftSS
    leg = legend([plot1,shadedPatch1,shadedPatch2], ...
        {'\tau_{MAP}','doubleSupport','leftSupport'},'Location','northeast','FontSize',18);
end
if flag_SSright % DS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','rightSupport'},'Location','northeast','FontSize',18);
end
if flag_SSleft & flag_SSright % DS + leftSS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch2,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','leftSupport','rightSupport'},'Location','northeast','FontSize',18);
end

subplotsqueeze(gcf, 1.2);
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('left_knee')),fig,600);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% RIGHT KNEE JOINT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for tauIdx = 1 : nrDofs
    % ----------------------- Right Knee rotx ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightKnee_rotx')
        torque_RK_rotx = estimatedVariables.tau.values(tauIdx,:);
        maxVal_RK_rotx = max(torque_RK_rotx);
        minVal_RK_rotx = min(torque_RK_rotx);
    end
    % ----------------------- Right Knee roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightKnee_roty')
        torque_RK_roty = estimatedVariables.tau.values(tauIdx,:);
        maxVal_RK_roty = max(torque_RK_roty);
        minVal_RK_roty = min(torque_RK_roty);
    end
    % ----------------------- Right Knee roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightKnee_rotz')
        torque_RK_rotz = estimatedVariables.tau.values(tauIdx,:);
        maxVal_RK_rotz = max(torque_RK_rotz);
        minVal_RK_rotz = min(torque_RK_rotz);
    end
end
%% Decide a ylim for all the rot
maxVal = max([maxVal_RK_rotx, maxVal_RK_roty, maxVal_RK_rotz]);
minVal = min([minVal_RK_rotx, minVal_RK_roty, minVal_RK_rotz]);

%% Plot
fig = figure('Name', 'Joint torque w.r.t. pattern detection','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

% Colors
doubleSupportColor       = [0.87058824300766 0.921568632125854 0.980392158031464];
singleSupportColor_left  = [1 0.800000011920929 0.800000011920929];
singleSupportColor_right = [0.756862759590149 0.866666674613953 0.776470601558685];

subplot (311) % Right Knee rotx ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_RK_rotx,'k','lineWidth',1.5);

title(sprintf('Right Knee Joint, Subj %s, Task %s',num2str(subjectID),num2str(taskID)));
ylabel('\tau_{x} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;


subplot (312) % Right Knee roty ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_RK_roty,'k','lineWidth',1.5);

ylabel('\tau_{y} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;

subplot (313) % Right Knee rotz ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_RK_rotz,'k','lineWidth',1.5);

ylabel('\tau_{z} [Nm]');
xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;

%legend
if flag_SSleft % DS + leftSS
    leg = legend([plot1,shadedPatch1,shadedPatch2], ...
        {'\tau_{MAP}','doubleSupport','leftSupport'},'Location','northeast','FontSize',18);
end
if flag_SSright % DS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','rightSupport'},'Location','northeast','FontSize',18);
end
if flag_SSleft & flag_SSright % DS + leftSS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch2,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','leftSupport','rightSupport'},'Location','northeast','FontSize',18);
end

subplotsqueeze(gcf, 1.2);
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('right_knee')),fig,600);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LEFT KNEE JOINT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for tauIdx = 1 : nrDofs
    % ----------------------- Left Knee rotx ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftKnee_rotx')
        torque_LK_rotx = estimatedVariables.tau.values(tauIdx,:);
        maxVal_LK_rotx = max(torque_LK_rotx);
        minVal_LK_rotx = min(torque_LK_rotx);
    end
    % ----------------------- Left Knee roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftKnee_roty')
        torque_LK_roty = estimatedVariables.tau.values(tauIdx,:);
        maxVal_LK_roty = max(torque_LK_roty);
        minVal_LK_roty = min(torque_LK_roty);
    end
    % ----------------------- Left Knee roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftKnee_rotz')
        torque_LK_rotz = estimatedVariables.tau.values(tauIdx,:);
        maxVal_LK_rotz = max(torque_LK_rotz);
        minVal_LK_rotz = min(torque_LK_rotz);
    end
end

%% Decide a ylim for all the rot
maxVal = max([maxVal_LK_rotx, maxVal_LK_roty, maxVal_LK_rotz]);
minVal = min([minVal_LK_rotx, minVal_LK_roty, minVal_LK_rotz]);

%% Plot
fig = figure('Name', 'Joint torque w.r.t. pattern detection','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

% Colors
doubleSupportColor       = [0.87058824300766 0.921568632125854 0.980392158031464];
singleSupportColor_left  = [1 0.800000011920929 0.800000011920929];
singleSupportColor_right = [0.756862759590149 0.866666674613953 0.776470601558685];

subplot (311) % Left Knee rotx ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_LK_rotx,'k','lineWidth',1.5);

title(sprintf('Left Knee Joint, Subj %s, Task %s',num2str(subjectID),num2str(taskID)));
ylabel('\tau_{x} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;


subplot (312) % Left Knee roty ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_LK_roty,'k','lineWidth',1.5);

ylabel('\tau_{y} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;

subplot (313) % Left Knee rotz ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_LK_rotz,'k','lineWidth',1.5);

ylabel('\tau_{z} [Nm]');
xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;

%legend
if flag_SSleft % DS + leftSS
    leg = legend([plot1,shadedPatch1,shadedPatch2], ...
        {'\tau_{MAP}','doubleSupport','leftSupport'},'Location','northeast','FontSize',18);
end
if flag_SSright % DS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','rightSupport'},'Location','northeast','FontSize',18);
end
if flag_SSleft & flag_SSright % DS + leftSS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch2,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','leftSupport','rightSupport'},'Location','northeast','FontSize',18);
end

subplotsqueeze(gcf, 1.2);
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('left_ankle')),fig,600);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% RIGHT HIP JOINT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for tauIdx = 1 : nrDofs
    % ----------------------- Right Hip rotx ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightHip_rotx')
        torque_RH_rotx = estimatedVariables.tau.values(tauIdx,:);
        maxVal_RH_rotx = max(torque_RH_rotx);
        minVal_RH_rotx = min(torque_RH_rotx);
    end
    % ----------------------- Right Hip roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightHip_roty')
        torque_RH_roty = estimatedVariables.tau.values(tauIdx,:);
        maxVal_RH_roty = max(torque_RH_roty);
        minVal_RH_roty = min(torque_RH_roty);
    end
    % ----------------------- Right Hip roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightHip_rotz')
        torque_RH_rotz = estimatedVariables.tau.values(tauIdx,:);
        maxVal_RH_rotz = max(torque_RH_rotz);
        minVal_RH_rotz = min(torque_RH_rotz);
    end
end
%% Decide a ylim for all the rot
maxVal = max([maxVal_RH_rotx, maxVal_RH_roty, maxVal_RH_rotz]);
minVal = min([minVal_RH_rotx, minVal_RH_roty, minVal_RH_rotz]);

%% Plot
fig = figure('Name', 'Joint torque w.r.t. pattern detection','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

% Colors
doubleSupportColor       = [0.87058824300766 0.921568632125854 0.980392158031464];
singleSupportColor_left  = [1 0.800000011920929 0.800000011920929];
singleSupportColor_right = [0.756862759590149 0.866666674613953 0.776470601558685];

subplot (311) % Right Hip rotx ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_RH_rotx,'k','lineWidth',1.5);

title(sprintf('Right Hip Joint, Subj %s, Task %s',num2str(subjectID),num2str(taskID)));
ylabel('\tau_{x} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;


subplot (312) % Right Hip roty ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_RH_roty,'k','lineWidth',1.5);

ylabel('\tau_{y} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;

subplot (313) % Right Hip rotz ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_RH_rotz,'k','lineWidth',1.5);

ylabel('\tau_{z} [Nm]');
xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;
%legend
if flag_SSleft % DS + leftSS
    leg = legend([plot1,shadedPatch1,shadedPatch2], ...
        {'\tau_{MAP}','doubleSupport','leftSupport'},'Location','northeast','FontSize',18);
end
if flag_SSright % DS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','rightSupport'},'Location','northeast','FontSize',18);
end
if flag_SSleft & flag_SSright % DS + leftSS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch2,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','leftSupport','rightSupport'},'Location','northeast','FontSize',18);
end

subplotsqueeze(gcf, 1.2);
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('right_hip')),fig,600);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LEFT HIP JOINT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for tauIdx = 1 : nrDofs
    % ----------------------- Left Hip rotx ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftHip_rotx')
        torque_LH_rotx = estimatedVariables.tau.values(tauIdx,:);
        maxVal_LH_rotx = max(torque_LH_rotx);
        minVal_LH_rotx = min(torque_LH_rotx);
    end
    % ----------------------- Left Hip roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftHip_roty')
        torque_LH_roty = estimatedVariables.tau.values(tauIdx,:);
        maxVal_LH_roty = max(torque_LH_roty);
        minVal_LH_roty = min(torque_LH_roty);
    end
    % ----------------------- Left Hip roty ----------------------------
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftHip_rotz')
        torque_LH_rotz = estimatedVariables.tau.values(tauIdx,:);
        maxVal_LH_rotz = max(torque_LH_rotz);
        minVal_LH_rotz = min(torque_LH_rotz);
    end
end

%% Decide a ylim for all the rot
maxVal = max([maxVal_LH_rotx, maxVal_LH_roty, maxVal_LH_rotz]);
minVal = min([minVal_LH_rotx, minVal_LH_roty, minVal_LH_rotz]);

%% Plot
fig = figure('Name', 'Joint torque w.r.t. pattern detection','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

% Colors
doubleSupportColor       = [0.87058824300766 0.921568632125854 0.980392158031464];
singleSupportColor_left  = [1 0.800000011920929 0.800000011920929];
singleSupportColor_right = [0.756862759590149 0.866666674613953 0.776470601558685];

subplot (311) % Left Hip rotx ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_LH_rotx,'k','lineWidth',1.5);

title(sprintf('Left Hip Joint, Subj %s, Task %s',num2str(subjectID),num2str(taskID)));
ylabel('\tau_{x} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;


subplot (312) % Left Hip roty ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_LH_roty,'k','lineWidth',1.5);

ylabel('\tau_{y} [Nm]');
% xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;

subplot (313) % Left Hip rotz ----------------------------
% DS
for shadedIdx = 1 : patternRanges.nrOfDS
    y1 = maxVal;
    y2 = minVal;
    x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
    x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
    shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    hold on
end

% SS_left
flag_SSleft = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
        flag_SSleft = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
    end
end
hold on

% SS_right
flag_SSright = false; % by default
for shadedIdx = 1 : patternRanges.nrOfDS
    if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
        flag_SSright = true;
        y1 = maxVal;
        y2 = minVal;
        x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
        shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
    end
end
hold on
plot1 = plot(torque_LH_rotz,'k','lineWidth',1.5);

ylabel('\tau_{z} [Nm]');
xlabel('samples');
set(gca,'FontSize',15)
grid on;
axis tight;

%legend
if flag_SSleft % DS + leftSS
    leg = legend([plot1,shadedPatch1,shadedPatch2], ...
        {'\tau_{MAP}','doubleSupport','leftSupport'},'Location','northeast','FontSize',18);
end
if flag_SSright % DS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','rightSupport'},'Location','northeast','FontSize',18);
end
if flag_SSleft & flag_SSright % DS + leftSS + rightSS
    leg = legend([plot1,shadedPatch1,shadedPatch2,shadedPatch3], ...
        {'\tau_{MAP}','doubleSupport','leftSupport','rightSupport'},'Location','northeast','FontSize',18);
end

subplotsqueeze(gcf, 1.2);
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('left_hip')),fig,600);
end

