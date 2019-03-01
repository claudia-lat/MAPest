
close all;
opts.plotPatternDetection = true;

%% Preliminaries
tmp.shoeLength = size(shoes.Left_HF,2);

% feet mean
shoes.meanValue = mean(shoes.Left_HF(3,:) + shoes.Right_HF(3,:))/2;
tmp.meanVectt = zeros(1, tmp.shoeLength);
tmp.meanVectt(1,:) = shoes.meanValue;
%
% subject weight
shoes.weight = mean(shoes.Left_HF(3,:) + shoes.Right_HF(3,:));
% weightVect = zeros(1, tmp.shoeLength);

%% Define ranges for double support (DS)
tmp.DSconditionWRTtotalLength = zeros(tmp.shoeLength,1);
for crossIdx = 1 : tmp.shoeLength
    if shoes.Left_HF(3,crossIdx) == shoes.Right_HF(3,crossIdx) || ...
            abs(shoes.Left_HF(3,crossIdx) - shoes.Right_HF(3,crossIdx)) <= (patternRanges.parameterForDStuning*shoes.weight)/100
        tmp.DSconditionWRTtotalLength(crossIdx,1) = crossIdx;
    end
end

tmp.intervalDS = diff(tmp.DSconditionWRTtotalLength);
tmp.indexDS = [ ];
for intervalIdx  = 1 : length(tmp.DSconditionWRTtotalLength)-1
    if tmp.intervalDS(intervalIdx) == 1
        tmp.indexDS = [tmp.indexDS,intervalIdx];
    end
end

tmp.rangeMin = [ ];
tmp.rangeMax = [ ];
diffVect = diff(tmp.indexDS);
for DSconditionIdx = 1 : length(tmp.indexDS)-1
    if diffVect(DSconditionIdx) ~= 1
        tmp.rangeMin = [tmp.rangeMin,tmp.indexDS(DSconditionIdx+1)];
        tmp.rangeMax = [tmp.rangeMax,tmp.indexDS(DSconditionIdx)];
    end
end
tmp.rangeMin = [1, tmp.rangeMin];
tmp.rangeMax = [tmp.rangeMax,tmp.indexDS(end)];

tmp.nrOfDS = length(tmp.rangeMin);
for DSidx = 1 : tmp.nrOfDS
    patternRanges.DSrange(DSidx).rangeMin = tmp.rangeMin(DSidx);
    patternRanges.DSrange(DSidx).rangeMax = tmp.rangeMax(DSidx);
    patternRanges.DSrange(DSidx).contact  = 'doubleSupport';
end

%% Define ranges for single support (SS)
for SSidx = 1 : tmp.nrOfDS
    patternRanges.SSrange(SSidx).rangeMin = patternRanges.DSrange(SSidx).rangeMax;
    if SSidx == tmp.nrOfDS(end)
        patternRanges.SSrange(SSidx).rangeMax = tmp.shoeLength;
    else
        patternRanges.SSrange(SSidx).rangeMax = patternRanges.DSrange(SSidx+1).rangeMin;
    end
end

%% If SS, define the foot in contact with the ground
for SSidx = 1 : length(patternRanges.SSrange)
    tmp.SSextractionLeftFoot  = mean(shoes.Left_HF(3,patternRanges.SSrange(SSidx).rangeMin:patternRanges.SSrange(SSidx).rangeMax));
    tmp.SSextractionRightFoot = mean(shoes.Right_HF(3,patternRanges.SSrange(SSidx).rangeMin:patternRanges.SSrange(SSidx).rangeMax));
    if tmp.SSextractionLeftFoot > tmp.SSextractionRightFoot
        % foot in contact: Left
        patternRanges.SSrange(SSidx).contact = 'LeftFoot';
    else
        % foot in contact: Right
        patternRanges.SSrange(SSidx).contact = 'RightFoot';
    end
end

%% If DS range is small, consider the previous SS
tmp.DSfieldTobeRemoved = [ ];
for DSidx = 1 : tmp.nrOfDS
    if (patternRanges.DSrange(DSidx).rangeMax - patternRanges.DSrange(DSidx).rangeMin) < patternRanges.sampleDSTreshold
        tmp.DSfieldTobeRemoved = [tmp.DSfieldTobeRemoved; patternRanges.DSrange(DSidx).rangeMin];
        
    end
end

for removeIdx = 1 : length(tmp.DSfieldTobeRemoved)
    for DSidx = 1 : tmp.nrOfDS
        if tmp.DSfieldTobeRemoved(removeIdx) == patternRanges.SSrange(DSidx).rangeMax
            patternRanges.SSrange(DSidx).rangeMax = patternRanges.SSrange(DSidx+1).rangeMin;
        end
    end
end

%% Final approximated contact pattern range
contactPattern = cell(tmp.shoeLength,1);
for lenIdx  = 1 : tmp.shoeLength
    for SSidx = 1 : length(patternRanges.SSrange)
        if lenIdx >= patternRanges.SSrange(SSidx).rangeMin && lenIdx <= patternRanges.SSrange(SSidx).rangeMax
            contactPattern{lenIdx} = patternRanges.SSrange(SSidx).contact;
            break
        else
            contactPattern{lenIdx} = 'doubleSupport';
        end
    end
end

patternRanges.nrOfDS = tmp.nrOfDS;

%% Plots
if opts.plotPatternDetection
    fig = figure('Name', 'Contact pattern detection','NumberTitle','off');
    axes1 = axes('Parent',fig,'FontSize',16);
    box(axes1,'on');
    hold(axes1,'on');
    grid on;
    ylim auto;
    % DS
    doubleSupportColor = [0.87058824300766 0.921568632125854 0.980392158031464];
    for shadedIdx = 1 : tmp.nrOfDS
        y1 = 0;
        y2 = 900;
        x1 = patternRanges.DSrange(shadedIdx).rangeMin(1);
        x2 = patternRanges.DSrange(shadedIdx).rangeMax(1);
        shadedPatch1 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],doubleSupportColor);
    end
    hold on
    
    % SS_left
    flag_SSleft = false; % by default
    singleSupportColor_left  = [1 0.800000011920929 0.800000011920929];
    for shadedIdx = 1 : tmp.nrOfDS
        if strcmp(patternRanges.SSrange(shadedIdx).contact,'LeftFoot')
            flag_SSleft = true;
            y1 = 0;
            y2 = 900;
            x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
            x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
            shadedPatch2 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_left);
        end
    end
    hold on
    
    % SS_right
    flag_SSright = false; % by default
    singleSupportColor_right = [0.756862759590149 0.866666674613953 0.776470601558685];
    for shadedIdx = 1 : tmp.nrOfDS
        if strcmp(patternRanges.SSrange(shadedIdx).contact,'RightFoot')
            flag_SSright = true;
            y1 = 0;
            y2 = 900;
            x1 = patternRanges.SSrange(shadedIdx).rangeMin(1);
            x2 = patternRanges.SSrange(shadedIdx).rangeMax(1);
            shadedPatch3 = fill([x1 x1 x2 x2],[y1 y2 y2 y1],singleSupportColor_right);
        end
    end
    hold on
    
    plot1 = plot(shoes.Left_HF(3,:),'r','lineWidth',1.5);
    hold on
    plot2 = plot(shoes.Right_HF(3,:),'Color',[0 0.498039215803146 0],'lineWidth',1.5);
    hold on
    plot3 = plot(tmp.meanVectt,'lineWidth',0.7,'LineStyle','--','Color',[0 0 0]);
    
    title(sprintf('Contact pattern detection, Subj %s, Task %s',num2str(subjectID),num2str(taskID)));
    ylabel('f_z [N]');
    xlabel('samples');
    set(gca,'FontSize',15)
    grid on;
    
    %legend
    if flag_SSleft % DS + leftSS
        leg = legend([plot1,plot2,plot3,shadedPatch1,shadedPatch2], ...
            {'Left','Right','mean','doubleSupport','leftSupport'},'Location','northeast','FontSize',18);
    end
    if flag_SSright % DS + rightSS
        leg = legend([plot1,plot2,plot3,shadedPatch1,shadedPatch3], ...
            {'Left','Right','mean','doubleSupport','rightSupport'},'Location','northeast','FontSize',18);
    end
    if flag_SSleft & flag_SSright % DS + leftSS + rightSS
        leg = legend([plot1,plot2,plot3,shadedPatch1,shadedPatch2,shadedPatch3], ...
            {'Left','Right','mean','doubleSupport','leftSupport','rightSupport'},'Location','northeast','FontSize',18);
    end
    set(leg,'Interpreter','latex');
    axis tight;
end

%% Clean up
clearvars tmp ...
    crossIdx DSconditionIdx DSidx SSidx shadedIdx intervalIdx removeIdx...
    shadedPatch1 shadedPatch2 shadedPatch3 ...
    flag_SSleft flag_SSright ...
    plot1 plot2 plot3 axes1...
    x1 x2 y1 y2 ...
    singleSupportColor_left singleSupportColor_right doubleSupportColor;

close all;
