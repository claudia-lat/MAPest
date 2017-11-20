function pos = extractForceplatesPositionWrtCortex(filename)
%EXTRACTFORCEPLATESPOSITIONWRTCORTEX reads the Cortex .trc file and provides
% vectors with:
%  - the 4 markers positioned on top of them (2 x forceplate);
%  - the reference frame of each forceplates.
% Every output of this function is expressed in m wrt the known Cortex 
% reference frame. 

%% Read the .trc file
fid = fopen(filename);
Rows = textscan(fid, '%s', 'delimiter', '\n'); % creates a temporary array with the rows
fclose(fid);

% Detect numbers in the rows, and discard 'VOID' elements
Columns= cellfun(@(x) textscan(x,'%f','delimiter','\t','CollectOutput',1,'treatAsEmpty',{'VOID'}), Rows{1,1});
Columns= cellfun(@transpose, Columns, 'UniformOutput', 0);

%% Extract the 4 markers positions
% The Cortex .trc file gives information on the 4 markers (labelled as 
% following) placed on the corners of the forceplates
TopRight    = zeros(size(Columns,1),3);
BottomRight = zeros(size(Columns,1),3);
BottomLeft  = zeros(size(Columns,1),3);
TopLeft     = zeros(size(Columns,1),3);
for i = 6 : size(Columns,1)
    TopRight(i-5,:)    = Columns{i,1}(1,3:5);
    BottomRight(i-5,:) = Columns{i,1}(1,6:8);
    BottomLeft(i-5,:)  = Columns{i,1}(1,9:11);
    TopLeft(i-5,:)     = Columns{i,1}(1,12:14);
end

%% Get the position vectors of the 4 markers
% Mean of the first 50 samples (since it is a static acqisition)
TopRight_mean    = [mean(TopRight(1:50,1)),    mean(TopRight(1:50,2)),    mean(TopRight(1:50,3))   ];
BottomRight_mean = [mean(BottomRight(1:50,1)), mean(BottomRight(1:50,2)), mean(BottomRight(1:50,3))];
BottomLeft_mean  = [mean(BottomLeft(1:50,1)),  mean(BottomLeft(1:50,2)),  mean(BottomLeft(1:50,3)) ];
TopLeft_mean     = [mean(TopLeft(1:50,1)),     mean(TopLeft(1:50,2)),     mean(TopLeft(1:50,3))    ];

% Check the 3th component of the 4 markers vector. This element represents
% the height of the forceplates wrt the Cortex reference frame.  It should
% be equal for all the acquisition.  Since markers are positioned on top of
% some sketches, probably these slightly modified this value --> use the
% little one for all!
heightValue = [TopRight_mean(3),BottomRight_mean(3), BottomLeft_mean(3), TopLeft_mean(3)];
heightValue_min = min(heightValue);
TopRight_mean(3)    = heightValue_min;
BottomRight_mean(3) = heightValue_min; 
BottomLeft_mean(3)  = heightValue_min;
TopLeft_mean(3)     = heightValue_min;

pos.markers.TopRight    = TopRight_mean;
pos.markers.BottomRight = BottomRight_mean;
pos.markers.BottomLeft  = BottomLeft_mean;
pos.markers.TopLeft     = TopLeft_mean;

%% Get the position vectors of the two forceplates reference frames
% We know that they are positioned on the middle-top surface, respectively.

% Forceplate size = 502 x 502 (from datasheet)
midPlatform = 502/2;

%FP1
midFP1x_viaTopRight    = TopRight_mean(1) + midPlatform;
midFP1x_viaBottomRight = BottomRight_mean(1) + midPlatform;
originFP1frame_x = (midFP1x_viaTopRight + midFP1x_viaBottomRight)/2;
originFP1frame_y = (TopRight_mean(2) + BottomRight_mean(2))/2;
originFP1frame_z = heightValue_min; %origin on top of the forceplate

%FP2
midFP2x_viaTopLeft    = TopLeft_mean(1) - midPlatform;
midFP2x_viaBottomLeft = BottomLeft_mean(1) - midPlatform;
originFP2frame_x = (midFP2x_viaTopLeft + midFP2x_viaBottomLeft)/2;
originFP2frame_y = (TopLeft_mean(2)+BottomLeft_mean(2))/2;
originFP2frame_z = heightValue_min; %origin on top of the forceplate

pos.FP1 = [originFP1frame_x originFP1frame_y originFP1frame_z].*1e-3; % in m
pos.FP2 = [originFP2frame_x originFP2frame_y originFP2frame_z].*1e-3; % in m

end
