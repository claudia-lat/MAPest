function [ forceplate, suitIndex ] = extractForceplateData(AMTIfilename, TSfilename, suitTimeInit, contactLink, varargin )
% EXTRACTFORCEPLATEDATA allows to create a .mat stucture contatining all 
% forceplate data acquired during the Xsens experiment.
%
% Inputs 
% -  AMTIfilename : the name of  file containing the forceplate data;
% -  TSfilename   : the name of  file containing the forceplate time data;
% -  suitTimeInit : time data from the xsens suit;
% -  contactLink  : human links in contact with the forceplates;
% -  outputDir    : (optional) the directory where saving the output;
% -  allData      : (optional) if TRUE the function returns not only the 
%                    cut data but all the data genereted by the forceplates.
%                    FALSE by default.
%
% Outputs
% -  forceplate : data of the acquisition in a .mat format; 
% -  XsensIndex : index of the xsens data that have a corresponding 
%                 forceplate data. 

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
AMTIData = getForceplateDataInMatrixForm (fullfile(pwd,AMTIfilename));
AMTIData = AMTIData(AMTIData(:,1) ~= 0, :);
AMTIData = AMTIData(2:end,:);
TSData = getForceplateDataInMatrixForm(fullfile(pwd,TSfilename));
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
allData.plateforms.plateform1.frames      = AMTIData(:,1)';
allData.plateforms.plateform1.contactLink = contactLink{1};
allData.plateforms.plateform1.forces      = AMTIData(:,2:4)';
allData.plateforms.plateform1.moments     = AMTIData(:,5:7)';
allData.plateforms.plateform2.frames      = AMTIData(:,9)';
allData.plateforms.plateform2.contactLink = contactLink{2};
allData.plateforms.plateform2.forces      = AMTIData(:,10:12)';
allData.plateforms.plateform2.moments     = AMTIData(:,13:15)';

%% Function to determine the forceplate data corresponding to the suit data
forceplateTime = allData.time.unixTime;
[AMTIindex, suitIndex] = timeCmp(suitTimeInit,forceplateTime, 1);
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
data.plateforms.plateform1.frames      = cutData(:,1)';
data.plateforms.plateform1.contactLink = contactLink{1};
data.plateforms.plateform1.forces      = cutData(:,2:4)';
data.plateforms.plateform1.moments     = cutData(:,5:7)';
data.plateforms.plateform2.frames      = cutData(:,9)';
data.plateforms.plateform2.contactLink = contactLink{2};
data.plateforms.plateform2.forces      = cutData(:,10:12)';
data.plateforms.plateform2.moments     = cutData(:,13:15)';

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

function [matrixData] =  getForceplateDataInMatrixForm (filename)
%GETFORCEPLATEDATAINMATRIXFORM generates a matrix data from a file txt that
% it is opened by using 'fopen' as a column vector.  
% Note: this fuction is used since Matlab function 'dlmread' does jnot work
% with some Matlab version!

fileID = fopen(filename,'r');
vectorData = fscanf(fileID, '%f');
% vectorData is a column vector containing forceplate data in which the
% first element is ALWAYS the number of rows of the output matrix, the 
% second element is ALWAYS the number of columns of the output matrix.

rows = vectorData(1,1);
columns = vectorData(2,1);
matrixData = zeros(rows, columns);
vectorData = vectorData(3:end,1); % cutted the first two elements
for i = 1 : rows  
matrixData(i,:) = vectorData(columns*(i-1)+1 : columns*i ,1);
end
end
