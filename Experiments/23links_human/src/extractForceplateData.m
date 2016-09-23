function [ forceplate, suitIndex ] = extractForceplateData(AMTIfilename, TSfilename, suitTimeInit, contactLink, varargin )
% EXTRACTFORCEPLATEDATA allows to create a .mat stucture contatining all forceplate data 
% acquired during the Xsens experiment.

% Inputs 
% -  AMTIfilename : the name of  file that contain the forceplate data;
% -  TSfilename : the name of  file that contain the forceplate time data;
% -  suitTimeInit : time data from the xsens suit;
% -  contactLink : links in contact with the forceplates;
% -  outputDir : (optional) the directory where saving the output;
% -  allData : (optional) if true the function returns not only the cut data but all the data genereted by the forceplates.

% Outputs
% -  forceplate : data of the acquisition in a .mat format; 
% -  XsensIndex : index of the xsens data that have a corresponding forceplate data. 

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
delimiterIn = '\t';
AMTIData = dlmread(AMTIfilename, delimiterIn);
AMTIData = AMTIData(AMTIData(:,1) ~= 0, :);
AMTIData = AMTIData(2:end,:);
TSData = dlmread(TSfilename, delimiterIn);
TSData = TSData(TSData(:,1) ~= 0, :);
TSData = TSData(2:(size(AMTIData,1)+1),:);
% Timeunix control
diffIdx = find(diff(TSData(1:size(TSData),2))<0);
diffSum = sum(diff(TSData(1:size(TSData),2))<0);
diffMatrix = [diffIdx diffSum];
if diffSum > 0
    save('data/error.txt','diffMatrix','-ascii');
    error('The difference between adjacent elements of forceplate.timeunix cannot be negative!')
end


%% Create data struct for all data coming from the forceplates
allData =[];

% PROPERTIES
allData.properties.frameRate = 1000;
allData.properties.nrOfPlateform = 2;
nrOfFrames = size(AMTIData,1);
allData.properties.nrOfFrame = nrOfFrames;

% TIME
allData.time.unixTime = TSData(:,2)';
allData.time.standardTime = TSData(:,3)';

% PLATEFORMS
allData.plateforms.plateform1.frames = AMTIData(:,1)';
allData.plateforms.plateform1.contactLink = contactLink{1};
allData.plateforms.plateform1.forces = AMTIData(:,2:4)';
allData.plateforms.plateform1.moments = AMTIData(:,5:7)';
allData.plateforms.plateform2.frames = AMTIData(:,9)';
allData.plateforms.plateform2.contactLink = contactLink{2};
allData.plateforms.plateform2.forces = AMTIData(:,10:12)';
allData.plateforms.plateform2.moments = AMTIData(:,13:15)';

%% Function to determine the forceplate data corresponding to the suit data
forceplateTime = allData.time.unixTime;
[AMTIindex, suitIndex] = timeCmp(suitTimeInit,forceplateTime);
AMTIindex = AMTIindex(AMTIindex(:,1) ~= 0, :);
cutData = AMTIData(AMTIindex,:);
cutTSData = TSData(AMTIindex,:);

%% Create data struct for cut data corresponding to the suit
data = [];

% PROPERTIES
data.properties.frameRate = 1000;
data.properties.nrOfPlateform = 2;
nrOfFrames = size(AMTIindex,1);
data.properties.nrOfFrame = nrOfFrames;

% TIME
data.time.unixTime = cutTSData(:,2)';
data.time.standardTime = cutTSData(:,3)';

% PLATEFORMS
data.plateforms.plateform1.frames = cutData(:,1)';
data.plateforms.plateform1.contactLink = contactLink{1};
data.plateforms.plateform1.forces = cutData(:,2:4)';
data.plateforms.plateform1.moments = cutData(:,5:7)';
data.plateforms.plateform2.frames = cutData(:,9)';
data.plateforms.plateform2.contactLink = contactLink{2};
data.plateforms.plateform2.forces = cutData(:,10:12)';
data.plateforms.plateform2.moments = cutData(:,13:15)';

%% Create data struct
forceplate = [];

if (options.ALLDATA == 1)
    forceplate.data = data;
    forceplate.allData = allData;
else 
    forceplate.data = data;
end


%% Save data in a file.mat
if not(isempty(options.OUTPUTDIR))
    filename = 'forceplateData.mat';
    dir = fullfile(pwd, options.OUTPUTDIR);
    if ~exist(dir,'dir')
        mkdir(dir);
    end
    save(fullfile(options.OUTPUTDIR, filename),'forceplate');
end
end

function [slaveIndex, masterIndex] = timeCmp(masterTime,slaveTime)
% TIMECMP: Function to synchronize two system with different sampling
% Input:
% - masterTime : time data of the system that impose the sampling;
% - slaveTime : time data of the system that have to be synchronized with the master system
% Output:
% - slaveIndex : index for the sample of the slave that have a corresponding master data, 
%                zero if there is not a slave sample that corresponds to the master;
% - masterIndex : index for the sample of the master that have a corresponding slave data.

lenMaster = length(masterTime);
lenSlave = length(slaveTime);
slaveIndex = zeros(lenMaster, 1);
match = zeros(lenMaster, 1);
masterIndex = [];
jumpFlag = 0;
for i=1:lenMaster
    for j=1:lenSlave
        if (slaveIndex(i)==0)
            if (isequal(round(masterTime(i)),round(slaveTime(j)))==1)
                slaveIndex(i)=j;
                masterIndex = [masterIndex; i];
            else          
                [value,index] = min(abs(round(masterTime(i))-round(slaveTime)));
                if value <= 1
                    slaveIndex(i) = index;
                    masterIndex = [masterIndex; i];
                end     
            end
        end
    end
end

    match = find(slaveIndex);
    for i = 2:length(match)
        if (match(i)-match(i-1))>1
            jumpFlag = 1;
        end
    end

end


