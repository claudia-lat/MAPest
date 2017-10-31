%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  THIS SCRIPT IS FOR THE SENSOR COMBINATION Xsens + forceplates  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Load forceplates measurements files 
forceplates = struct;

bucket.forceplate_fileANC = sprintf(fullfile(bucket.pathToTrial,...
    '/forceplates/exercise%d.anc'), trialID);
bucket.forceplate_Offset = fullfile(bucket.pathToTrial,...
    '/forceplates/unloaded_fp1.anc');

%----------------------------------------------------------------------
% IMPORTANT NOTE:
% ---------------
% The file .anc (one for both forceplates) is extracted by the mocap 
% (Cortex) system and provides raw data only in voltages!  The parser 
% function convert voltages data in N and Nm!
%----------------------------------------------------------------------

%% Parse forceplates measurements
[forceplates.FP1, forceplates.FP2] = parseForceplates(bucket.forceplate_fileANC, ...
                                                      bucket.forceplate_Offset);
 
%% Synchronize data within between the two forcwplates

% TODO: synchronize measurements between two forceplates

%% Synchronize shoes and suit  
% At this stage:
% - data from forceplate are acquired at 100Hz, data from Xsens at 240Hz
% - suit acquisition started when triggered by the cortex system that 
%   simoultaneously triggered the forceplates.  Ideally xsens and
%   forceplates start and end at the same time!


%% Extract subject weight
% The weight of the subject is the sum of forceplates forces minus the
% weight of the shoes! 


%  bucket.weight = (forceplate.data.plateforms.plateform1.forces(3,1) ...
%                        + forceplate.data.plateforms.plateform2.forces(3,1))/9.81;
% peso meno le scape


