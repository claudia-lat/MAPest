%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  THIS SCRIPT IS FOR THE SENSOR COMBINATION Xsens + forceplates  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Load forceplates measurements files 
forceplates = struct;

bucket.forceplate_fileANC = sprintf(fullfile(bucket.pathToTrial,...
    '/forceplates/exercise%d.anc'), trialID);
bucket.forceplate_Offset = fullfile(bucket.pathToTrial,...
    '/forceplates/unloaded_fp1.anc');

%% Parse forceplates measurements
%----------------------------------------------------------------------
% IMPORTANT NOTE:
% ---------------
% The file .anc (one for both forceplates) is extracted by the mocap 
% (Cortex) system and provides raw data only in voltages!  The parser 
% function convert voltages data in N and Nm!
%----------------------------------------------------------------------
% Important assumption: data coming from the parser are already
% synchronized between them!

[forceplates.time, forceplates.FP1, forceplates.FP2] =  ...
                            parseForceplates(bucket.forceplate_fileANC, ...
                                             bucket.forceplate_Offset);
 
%% Synchronize forceplates and suit  
% At this stage:
% - data from forceplate are acquired at 100Hz, data from Xsens at 240Hz
% - suit acquisition started when triggered by the cortex system that 
%   simoultaneously triggered the forceplates.  Ideally xsens and
%   forceplates start and end at the same time!

% modified tmp the suit time fro relative to absolute

suit_time_rel = suit.time .* 1.e-3; %to ms.* 1.e-3; %to ms
suit_time_abs = zeros(size(suit_time_rel));
for i = 1 : size(suit_time_rel,2)
    suit_time_abs(:,i) = suit_time_rel(:,i) - suit_time_rel(:,1);
end


test =1;
%% Extract subject weight
% The weight of the subject is the sum of forceplates forces minus the
% weight of the shoes! 

% weight_from_FP = (abs(mean(forceplate.data.plateforms.plateform1.forces(3,1),'omitnan')) ...
%                 + abs(meanforceplate.data.plateforms.plateform2.forces(3,1),'omitnan')))/9.81;
% weight_shoes   = xxx;
% bucket.weight = weight_from_FP - weight_shoes;

