function [robot , syncIndex] = extractRobotData(ROBOTfilenameLeft, ...
                                                ROBOTfilenameRight, ...
                                                timeSeries,...
                                                contactLink,...
                                                rightArmStateFilename,...
                                                leftArmStateFilename,  ...
                                                rightLegStateFilename,...
                                                leftLegStateFilename, ...
                                                torsoStateFilename,...
                                                varargin )
% EXTRACTROBOTDATA allows to create a .mat stucture contatining all robot data 
% acquired during the human-robot interaction experiment.
%
% Inputs 
% -  ROBOTfilename  : (both Left and Right) filenames containing the FT 
%                     data from the robot;
% -  timeSeries     : time data to which we interpolate;
% -  contactLink    : human links in contact with the robot;
% - ...stateFilename: (both Left and Right) files containing state data
%                     from robot.
% -  outputDir      : (optional) the directory where saving the output;
% -  allData        : (optional) if true the function returns not only the
%                    cut data but all the data genereted by the robot.
%
% Outputs
% -  robot     : data of the acquisition in a .mat format;
% -  syncIndex : index of the Xsens data that correspond to robot data.


options = struct(   ...
    'OUTPUTDIR', '',...
    'ALLDATA',  true... 
    );
% read the acceptable names
optionNames = fieldnames(options);

% count arguments
nArgs = length(varargin);
if round(nArgs/2)~=nArgs/2
    error('number of input is wrong')
end

for pair = reshape(varargin,2,[]) % pair is {propName;propValue}
    inpName = upper(pair{1}); % make case insensitive

    if any(strcmp(inpName,optionNames))
        % overwrite options. If you want you can test for the right class here
        % Also, if you find out that there is an option you keep getting wrong,
        % you can use "if strcmp(inpName,'problemOption'),testMore,end"-statements
        options.(inpName) = pair{2};
    else
        error('%s is not a recognized parameter name',inpName)
    end
end

%% Load and read file .txt
robotDatasetRight = importdata(fullfile(pwd,ROBOTfilenameRight));
robotTimeRight = double(int64(robotDatasetRight(:,2)*1000));
robotDataRight = robotDatasetRight(:,3:end);

robotDatasetLeft = importdata(fullfile(pwd,ROBOTfilenameLeft));
robotTimeLeft = double(int64(robotDatasetLeft(:,2)*1000));
robotDataLeft = robotDatasetLeft(:,3:end);

nrOfCol = size(robotDataLeft,2);

% Set the degrees of freedom of the part of the robot
armDoF = 16;
legDoF = 6;
torsoDoF = 3; 

[rightArmState, ~, ~, rightArmStateTime]  = readStateExt(armDoF,rightArmStateFilename); % state in deg
[leftArmState, ~, ~, leftArmStateTime]    = readStateExt(armDoF,leftArmStateFilename);  % state in deg
[rightLegState, ~, ~, rightLegStateTime]  = readStateExt(legDoF,rightLegStateFilename); % state in deg
[leftLegState, ~, ~, leftLegStateTime]    = readStateExt(legDoF,leftLegStateFilename);  % state in deg
[torsoState, ~, ~, torsoStateTime]        = readStateExt(torsoDoF,torsoStateFilename);  % state in deg

rightArmState = pi/180 .* rightArmState; % in rad
leftArmState  = pi/180 .* leftArmState;  % in rad
rightLegState = pi/180 .* rightLegState; % in rad
leftLegState  = pi/180 .* leftLegState;  % in rad
torsoState    = pi/180 .* torsoState;    % in rad

rightArmStateTime = double(int64(rightArmStateTime*1000));
leftArmStateTime = double(int64(leftArmStateTime*1000));
rightLegStateTime = double(int64(rightLegStateTime*1000));
leftLegStateTime  = double(int64(leftLegStateTime*1000));
torsoStateTime    = double(int64(torsoStateTime*1000));

%% Cut Data

% Synchronize right and left robot Forces&Moments data

    % This step would possibly remove data not matched between left and right
    % [tIndexLeft, tIndexRight] = timeCmp(robotTimeRight, robotTimeLeft, 1);
    % tIndexLeft = tIndexLeft(tIndexLeft(:,1) ~= 0, :);
    % 
    % robotTimeRight = (robotTimeRight(tIndexRight(1):tIndexRight(end)));
    % robotTimeLeft = (robotTimeLeft(tIndexLeft(1):tIndexLeft(end)));
    % 
    % robotDataRight = robotDataRight(tIndexRight(1):tIndexRight(end),:);
    % robotDataLeft = robotDataLeft(tIndexLeft(1):tIndexLeft(end),:);

% Synchronize robot state and robot data
timeInit = max([rightArmStateTime(1),leftArmStateTime(1),rightLegStateTime(1),leftLegStateTime(1),torsoStateTime(1),robotTimeRight(1),robotTimeLeft(1)]); 
timeFinal = min([rightArmStateTime(end),leftArmStateTime(end),rightLegStateTime(end),leftLegStateTime(end),torsoStateTime(end),robotTimeRight(end),robotTimeLeft(end)]);
threshold = 5; % [ms] maximum difference between two time values that make the synchronization acceptable;

rightArmState = rightArmState(:,(find(rightArmStateTime>=(timeInit-threshold),1)):(find(rightArmStateTime>=(timeFinal-threshold),1)));
rightArmStateTime = rightArmStateTime((find(rightArmStateTime>=(timeInit-threshold),1)):(find(rightArmStateTime>=(timeFinal-threshold),1)));

leftArmState = leftArmState(:,(find(leftArmStateTime>=(timeInit-threshold),1)):(find(leftArmStateTime>=(timeFinal-threshold),1)));
leftArmStateTime = leftArmStateTime((find(leftArmStateTime>=(timeInit-threshold),1)):(find(leftArmStateTime>=(timeFinal-threshold),1)));

rightLegState = rightLegState(:,(find(rightLegStateTime>=(timeInit-threshold),1)):(find(rightLegStateTime>=(timeFinal-threshold),1)));
rightLegStateTime = rightLegStateTime((find(rightLegStateTime>=(timeInit-threshold),1)):(find(rightLegStateTime>=(timeFinal-threshold),1)));

leftLegState = leftLegState(:,(find(leftLegStateTime>=(timeInit-threshold),1)):(find(leftLegStateTime>=(timeFinal-threshold),1)));
leftLegStateTime = leftLegStateTime((find(leftLegStateTime>=(timeInit-threshold),1)):(find(leftLegStateTime>=(timeFinal-threshold),1)));

torsoState = torsoState(:,(find(torsoStateTime>=(timeInit-threshold),1)):(find(torsoStateTime>=(timeFinal-threshold),1)));
torsoStateTime = torsoStateTime((find(torsoStateTime>=(timeInit-threshold),1)):(find(torsoStateTime>=(timeFinal-threshold),1)));

robotDataRight = robotDataRight((find(robotTimeRight>=(timeInit-threshold),1)):(find(robotTimeRight>=(timeFinal-threshold),1)),:);
robotTimeRight = robotTimeRight((find(robotTimeRight>=(timeInit-threshold),1)):(find(robotTimeRight>=(timeFinal-threshold),1)),:);

robotDataLeft = robotDataLeft((find(robotTimeLeft>=(timeInit-threshold),1)):(find(robotTimeLeft>=(timeFinal-threshold),1)),:);
robotTimeLeft = robotTimeLeft((find(robotTimeLeft>=(timeInit-threshold),1)):(find(robotTimeLeft>=(timeFinal-threshold),1)),:);

% Time matching between right and left robot data 
for i = 1:nrOfCol
    vec = robotDataLeft(:,i);
    robotDataLeftINT(:,i) = interp1(robotTimeLeft,vec,robotTimeRight,'linear');
end
for i = 1:armDoF
    vec = rightArmState(i,:);
    rightArmStateINT(i,:) = interp1(rightArmStateTime,vec,robotTimeRight,'linear');
end
for i = 1:armDoF
    vec = leftArmState(i,:);
    leftArmStateINT(i,:) = interp1(leftArmStateTime,vec,robotTimeRight,'linear');
end
for i = 1:legDoF
    vec = rightLegState(i,:);
    rightLegStateINT(i,:) = interp1(rightLegStateTime,vec,robotTimeRight,'linear');
end
for i = 1:legDoF
    vec = leftLegState(i,:);
    leftLegStateINT(i,:) = interp1(leftLegStateTime,vec,robotTimeRight,'linear');
end
for i = 1:torsoDoF
    vec = torsoState(i,:);
    torsoStateINT(i,:) = interp1(torsoStateTime,vec,robotTimeRight,'linear');
end
robotTime = robotTimeRight;

%% Create data struct for all data coming from the robot
allData = [];
% PROPERTIES
allData.properties.frameRate = 240;
nrOfFrames = size(robotDataRight,1);
allData.properties.nrOfFrame = nrOfFrames;
% TIME
allData.time.unixTime = robotTime';
% ROBOT DATA
allData.links.rightarm.forces = robotDataRight(:,1:3)';
allData.links.rightarm.moments = robotDataRight(:,4:6)';
allData.links.rightarm.contactLink = contactLink{3};
allData.links.leftarm.forces = robotDataLeftINT(:,1:3)';
allData.links.leftarm.moments = robotDataLeftINT(:,4:6)';
allData.links.leftarm.contactLink = contactLink{4};
% ROBOT JOINT POSITION
allData.q.rightArm = rightArmStateINT;
allData.q.leftArm  = leftArmStateINT;
allData.q.rightLeg = rightLegStateINT;
allData.q.leftLeg  = leftLegStateINT;
allData.q.torso    = torsoStateINT;

%% Sychronization with the suit data
[robotIndex, syncIndex] = timeCmp(timeSeries, robotTime, 6);
syncTime = timeSeries(syncIndex)';
robotIndex = robotIndex(robotIndex(:,1) ~= 0, :);
robotDataRight = robotDataRight(robotIndex(1):robotIndex(end),:);
robotDataLeftINT = robotDataLeftINT(robotIndex(1):robotIndex(end),:);
rightArmStateINT = rightArmStateINT(:,robotIndex(1):robotIndex(end));
leftArmStateINT = leftArmStateINT(:,robotIndex(1):robotIndex(end));
rightLegStateINT = rightLegStateINT(:,robotIndex(1):robotIndex(end));
leftLegStateINT = leftLegStateINT(:,robotIndex(1):robotIndex(end));
torsoStateINT = torsoStateINT(:,robotIndex(1):robotIndex(end));
robotTime = robotTime(robotIndex(1):robotIndex(end),:);
    
%% Interpolation
for i = 1:nrOfCol
    vec = robotDataRight(:,i);
    robotCutDataRight(:,i) = interp1(robotTime,vec,syncTime,'linear');
end
for i = 1:nrOfCol
    vec = robotDataLeftINT(:,i);
    robotCutDataLeft(:,i) = interp1(robotTime,vec,syncTime,'linear');
end
for i = 1:armDoF
    vec = rightArmStateINT(i,:);
    rightArmStateCut(i,:) = interp1(robotTime,vec,syncTime,'linear');
end
for i = 1:armDoF
    vec = leftArmStateINT(i,:);
    leftArmStateCut(i,:) = interp1(robotTime,vec,syncTime,'linear');
end
for i = 1:legDoF
    vec = rightLegStateINT(i,:);
    rightLegStateCut(i,:) = interp1(robotTime,vec,syncTime,'linear');
end
for i = 1:legDoF
    vec = leftLegStateINT(i,:);
    leftLegStateCut(i,:) = interp1(robotTime,vec,syncTime,'linear');
end
for i = 1:torsoDoF
    vec = torsoStateINT(i,:);
    torsoStateCut(i,:) = interp1(robotTime,vec,syncTime,'linear');
end
syncIndex = syncIndex(2:end-1); % first and last sample of the interpolation are not reliable
syncTime = syncTime(syncIndex);
robotCutDataRight = robotCutDataRight(syncIndex,:);
robotCutDataLeft = robotCutDataLeft(syncIndex,:);
rightArmStateCut = rightArmStateCut(:,syncIndex);
leftArmStateCut = leftArmStateCut(:,syncIndex);
rightLegStateCut = rightLegStateCut(:,syncIndex);
leftLegStateCut = leftLegStateCut(:,syncIndex);
torsoStateCut = torsoStateCut(:,syncIndex);

%% Create data struct for cut data corresponding to the suit
data = [];
% PROPERTIES
data.properties.frameRate = 240;
nrOfFrames = size(robotCutDataRight,1);
data.properties.nrOfFrame = nrOfFrames;
% TIME
data.time.unixTime = syncTime';
% DATA
data.links.rightarm.forces      = robotCutDataRight(:,1:3)';
data.links.rightarm.moments     = robotCutDataRight(:,4:6)';
data.links.rightarm.contactLink = contactLink{3};
data.links.leftarm.forces       = robotCutDataLeft(:,1:3)';
data.links.leftarm.moments       = robotCutDataLeft(:,4:6)';
data.links.leftarm.contactLink  = contactLink{4};
% ROBOT JOINT POSITION
data.q.rightArm = rightArmStateCut;
data.q.leftArm  = leftArmStateCut;
data.q.rightLeg = rightLegStateCut;
data.q.leftLeg  = leftLegStateCut;
data.q.torso    = torsoStateCut;
%% Create data struct
robot = [];
if (options.ALLDATA == 1)
    robot.data = data;
    robot.allData = allData;
else 
    robot.data = data;
end

%% Save data in a file.mat
if not(isempty(options.OUTPUTDIR))
    filename = 'robotData.mat';
    dir = fullfile(pwd, options.OUTPUTDIR);
    if ~exist(dir,'dir')
        mkdir(dir);
    end
    save(fullfile(options.OUTPUTDIR, filename),'robot');
end
end
