
%% Preliminaries
lenForces = size(shoes.Left_HF,2);

% feet mean
shoes.meanValue = mean(shoes.Left_HF(3,:) + shoes.Right_HF(3,:))/2;
meanVect = zeros(1, lenForces);
meanVect(1,:) = shoes.meanValue;

% subject weight
shoes.weight = mean(shoes.Left_HF(3,:) + shoes.Right_HF(3,:));
weightVect = zeros(1, lenForces);
weightVect(1,:) = shoes.weight;

%% Definition of the thresholds
% max limit for double-support stance phase (DS)
shoes.thresholdMaxDS = (65*shoes.weight)/100; % 75% of the weight
thresholdMaxDSVect = zeros(1, lenForces);
thresholdMaxDSVect(1,:) = shoes.thresholdMaxDS;

% min limit for double-support stance phase (DS)
shoes.thresholdMinDS = (35*shoes.weight)/100; % 25% of the weight
thresholdMinDSVect = zeros(1, lenForces);
thresholdMinDSVect(1,:) = shoes.thresholdMinDS;

%% Finding the indices crossing the thresholds
% -------Left
% Find indices when the left fz is crossing positively the thresholdMaxDS
crossingPosMaxDSidx_left = [ ];
for lenIdx  = 1 : lenForces-1
    if shoes.Left_HF(3,lenIdx) <= shoes.thresholdMaxDS  && shoes.Left_HF(3,lenIdx+1) >= shoes.thresholdMaxDS
        crossingPosMaxDSidx_left = [crossingPosMaxDSidx_left,lenIdx];
    end  
end
% Find indices when the left fz is crossing negatively the thresholdMaxDS
crossingNegMaxDSidx_left = [ ];
for lenIdx  = 1 : lenForces-1
    if shoes.Left_HF(3,lenIdx) >= shoes.thresholdMaxDS  && shoes.Left_HF(3,lenIdx+1) <= shoes.thresholdMaxDS
        crossingNegMaxDSidx_left = [crossingNegMaxDSidx_left,lenIdx];
    end  
end
% Find indices when the left fz is crossing positively the thresholdMinDS
crossingPosMinDSidx_left = [ ];
for lenIdx  = 1 : lenForces-1
    if shoes.Left_HF(3,lenIdx) <= shoes.thresholdMinDS  && shoes.Left_HF(3,lenIdx+1) >= shoes.thresholdMinDS
        crossingPosMinDSidx_left = [crossingPosMinDSidx_left,lenIdx];
    end  
end
% Find indices when the left fz is crossing negatively the thresholdMinDS
crossingNegMinDSidx_left = [ ];
for lenIdx  = 1 : lenForces-1
    if shoes.Left_HF(3,lenIdx) >= shoes.thresholdMinDS  && shoes.Left_HF(3,lenIdx+1) <= shoes.thresholdMinDS
        crossingNegMinDSidx_left = [crossingNegMinDSidx_left,lenIdx];
    end  
end

% -------Right
% Find indices when the rigth fz is crossing positively the thresholdMaxDS
crossingPosMaxDSidx_right = [ ];
for lenIdx  = 1 : lenForces-1
    if shoes.Right_HF(3,lenIdx) <= shoes.thresholdMaxDS  && shoes.Right_HF(3,lenIdx+1) >= shoes.thresholdMaxDS
        crossingPosMaxDSidx_right = [crossingPosMaxDSidx_right,lenIdx];
    end  
end
% Find indices when the rigth fz is crossing negatively the thresholdMaxDS
crossingNegMaxDSidx_right = [ ];
for lenIdx  = 1 : lenForces-1
    if shoes.Right_HF(3,lenIdx) >= shoes.thresholdMaxDS  && shoes.Right_HF(3,lenIdx+1) <= shoes.thresholdMaxDS
        crossingNegMaxDSidx_right = [crossingNegMaxDSidx_right,lenIdx];
    end  
end
% Find indices when the rigth fz is crossing positively the thresholdMinDS
crossingPosMinDSidx_right = [ ];
for lenIdx  = 1 : lenForces-1
    if shoes.Right_HF(3,lenIdx) <= shoes.thresholdMinDS  && shoes.Right_HF(3,lenIdx+1) >= shoes.thresholdMinDS
        crossingPosMinDSidx_right = [crossingPosMinDSidx_right,lenIdx];
    end  
end
% Find indices when the rigth fz is crossing negatively the thresholdMinDS
crossingNegMinDSidx_right = [ ];
for lenIdx  = 1 : lenForces-1
    if shoes.Right_HF(3,lenIdx) >= shoes.thresholdMinDS  && shoes.Right_HF(3,lenIdx+1) <= shoes.thresholdMinDS
        crossingNegMinDSidx_right = [crossingNegMinDSidx_right,lenIdx];
    end  
end

%% Plots
close all;

% maximum area of the signal
maxSignalRange = max(max(shoes.Left_HF(3,:)), max(shoes.Right_HF(3,:)));
maxSignalRangeVect = zeros(1, lenForces);
maxSignalRangeVect(1,:) = maxSignalRange;

% minimum area of the signal
minSignalRange = min(min(shoes.Left_HF(3,:)), min(shoes.Right_HF(3,:)));
minSignalRangeVect = zeros(1, lenForces);
minSignalRangeVect(1,:) = minSignalRange;

fig = figure('Name', 'WalkingExample','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;
ylim auto;

%STANCE PHASE: single support phase
singleSupportColor = [0.960784316062927 0.921568632125854 0.921568632125854];
y1 = maxSignalRange;
y2 = shoes.thresholdMaxDS;
x1 = 0;
x2 = lenForces;
shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor);
hold on

% STANCE PHASE: double support phase
doubleSupportColor = [0.87058824300766 0.921568632125854 0.980392158031464];
y1 = shoes.thresholdMinDS;
y2 = shoes.thresholdMaxDS;
x1 = 0;
x2 = lenForces;
shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
hold on

% STANCE PHASE: single support phase
singleSupportColor = [0.960784316062927 0.921568632125854 0.921568632125854];
y1 = minSignalRange;
y2 = shoes.thresholdMinDS;
x1 = 0;
x2 = lenForces;
shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor);
hold on

%plot on z axis
plot1 = plot(shoes.Left_HF(3,:),'lineWidth',1.5);
hold on
plot2 = plot(shoes.Right_HF(3,:),'lineWidth',1.5);
% hold on
% plot3 = plot(meanVect,'lineWidth',1);
% hold on
% plot4 = plot(maxSignalRangeVect,'lineWidth',1);
hold on
plot3 = plot(thresholdMaxDSVect,'lineWidth',1);
hold on
plot4 = plot(thresholdMinDSVect,'lineWidth',1);
% hold on
% plot7 = plot(minSignalRangeVect,'lineWidth',1);
ylim auto

% Vertical dashedlines
% ---left
for poslineIdx = 1 : size(crossingPosMaxDSidx_left,2)
    line([crossingPosMaxDSidx_left(poslineIdx) crossingPosMaxDSidx_left(poslineIdx)],[ylim],'LineStyle','--')
end
for neglineIdx = 1 : size(crossingNegMaxDSidx_left,2)
    line([crossingNegMaxDSidx_left(neglineIdx) crossingNegMaxDSidx_left(neglineIdx)],[ylim],'LineStyle','--')
end
for poslineIdx = 1 : size(crossingPosMinDSidx_left,2)
    line([crossingPosMinDSidx_left(poslineIdx) crossingPosMinDSidx_left(poslineIdx)],[ylim],'LineStyle','--')
end
for neglineIdx = 1 : size(crossingNegMinDSidx_left,2)
    line([crossingNegMinDSidx_left(neglineIdx) crossingNegMinDSidx_left(neglineIdx)],[ylim],'LineStyle','--')
end
% ---right
for poslineIdx = 1 : size(crossingPosMaxDSidx_right,2)
    line([crossingPosMaxDSidx_right(poslineIdx) crossingPosMaxDSidx_right(poslineIdx)],[ylim],'LineStyle','--')
end
for neglineIdx = 1 : size(crossingNegMaxDSidx_right,2)
    line([crossingNegMaxDSidx_right(neglineIdx) crossingNegMaxDSidx_right(neglineIdx)],[ylim],'LineStyle','--')
end
for poslineIdx = 1 : size(crossingPosMinDSidx_right,2)
    line([crossingPosMinDSidx_right(poslineIdx) crossingPosMinDSidx_right(poslineIdx)],[ylim],'LineStyle','--')
end
for neglineIdx = 1 : size(crossingNegMinDSidx_right,2)
    line([crossingNegMinDSidx_right(neglineIdx) crossingNegMinDSidx_right(neglineIdx)],[ylim],'LineStyle','--')
end

title('Shoes forces f_z w.r.t. human frames');
ylabel('f_z [N]');
set(gca,'FontSize',15)
grid on;
%legend
leg = legend([plot1,plot2,plot3,plot4,shadedPatch1,shadedPatch2,shadedPatch3], ...
    {'Left','Right','max DS','min DS','singleSupport','doubleSupport'},'Location','northeast');
set(leg,'Interpreter','latex');
