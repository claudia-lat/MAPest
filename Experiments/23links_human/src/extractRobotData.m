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
% -  contactLink    : links in contact with the robot;
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
    'ALLDATA',  false... 
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
robotTimeRight = (robotDatasetRight(:,2)*1000);

robotDatasetLeft = importdata(fullfile(pwd,ROBOTfilenameLeft));
robotTimeLeft = (robotDatasetLeft(:,2)*1000);

[tIndexLeft, tIndexRight] = timeCmp(robotTimeRight, robotTimeLeft, 1);
tIndexLeft = tIndexLeft(tIndexLeft(:,1) ~= 0, :);

robotTimeRight = double(int64(robotTimeRight(tIndexRight(1):tIndexRight(end))));
robotTimeLeft = double(int64(robotTimeLeft(tIndexLeft(1):tIndexLeft(end))));

robotDataRight = robotDatasetRight(tIndexRight(1):tIndexRight(end),3:end);
robotDataLeft = robotDatasetLeft(tIndexLeft(1):tIndexLeft(end),3:end);

% Time matching between right and left robot data 
for i = 1:6
    vec = robotDataLeft(:,i);
    robotDataLeft(:,i) = interp1(robotTimeLeft,vec,robotTimeRight,'linear');
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
allData.links.leftarm.forces = robotDataLeft(:,1:3)';
allData.links.leftarm.moments = robotDataLeft(:,4:6)';
allData.links.leftarm.contactLink = contactLink{4};

%% Sychronization with the suit data
[robotIndex, syncIndex] = timeCmp(timeSeries, robotTime, 6);
syncTime = timeSeries(syncIndex)';
robotDataRight = robotDataRight(robotIndex(1):robotIndex(end),:);
robotDataLeft = robotDataLeft(robotIndex(1):robotIndex(end),:);
robotTime = robotTime(robotIndex(1):robotIndex(end),:);
    
%% Interpolation
for i = 1:6
    vec = robotDataRight(:,i);
    robotCutDataRight(:,i) = interp1(robotTime,vec,syncTime,'linear');
end
for i = 1:6
    vec = robotDataLeft(:,i);
    robotCutDataLeft(:,i) = interp1(robotTime,vec,syncTime,'linear');
end
syncIndex = syncIndex(2:end-1); % first and last sample of the interpolation are not reliable
syncTime = syncTime(syncIndex);
robotCutDataRight = robotCutDataRight(syncIndex,:);
robotCutDataLeft = robotCutDataLeft(syncIndex,:);

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
data.links.leftarm.moment       = robotCutDataLeft(:,4:6)';
data.links.leftarm.contactLink  = contactLink{4};
% ROBOT JOINT POSITION
data.q.rightArm.state = readStateExt(16,rightArmStateFilename);
data.q.leftArm.state  = readStateExt(16,leftArmStateFilename);
data.q.rightLeg.state = readStateExt(6,rightLegStateFilename);
data.q.leftLeg.state  = readStateExt(6,leftLegStateFilename);
data.q.torso.state    = readStateExt(3,torsoStateFilename);
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
