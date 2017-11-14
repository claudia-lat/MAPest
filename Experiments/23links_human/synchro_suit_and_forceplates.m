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
% - data from forceplates are acquired at 100Hz, data from Xsens at 240Hz
% - suit acquisition started when triggered by the cortex system that 
%   simoultaneously triggered the forceplates.  Ideally Xsens and
%   forceplates start and end at the same time!

% Modified tmp the suit time from relative to absolute
suit_time_rel = suit.time .* 1.e-3; %to ms.* 1.e-3; %to ms
suit_time_abs = zeros(size(suit_time_rel));
for i = 1 : size(suit_time_rel,2)
    suit_time_abs(:,i) = suit_time_rel(:,i) - suit_time_rel(:,1);
end

% The two last samples of suit_time_abs and forceplates.time does not 
% coincides perfectly, probably due to the network lag. Before
% interpolating (upsampling) forceplates, it is needed to modify the
% forceplates time --> newTime that ends with suit_time_abs(end).
forceplates.tmp.timeToSuit = (0:0.01:suit_time_abs(end));

% ------------ STEP 1 
% Upsampling forceplates with the new time forceplates.tmp.timeToSuit
for i = 1 : size(forceplates.FP1.wrenches,2)
    forceplates.tmp.FP1.wrenches(:,i) = interp1(forceplates.time, ...
                                         forceplates.FP1.wrenches(:,i), ...
                                         forceplates.tmp.timeToSuit);
    forceplates.tmp.FP2.wrenches(:,i) = interp1(forceplates.time, ...
                                         forceplates.FP2.wrenches(:,i), ...
                                         forceplates.tmp.timeToSuit);
end 

% ------------ STEP 1 
% Upsampling (upsampled) forceplates with the suit
for i = 1 : size(forceplates.FP1.wrenches,2)
    forceplates.upsampled.FP1.wrenches(:,i) = interp1(forceplates.tmp.timeToSuit, ...
                                         forceplates.tmp.FP1.wrenches(:,i), ...
                                         suit_time_abs);
    forceplates.upsampled.FP2.wrenches(:,i) = interp1(forceplates.tmp.timeToSuit, ...
                                         forceplates.tmp.FP2.wrenches(:,i), ...
                                         suit_time_abs);
end

forceplates.upsampled.time = suit_time_abs; % new time for the upsampled forceplates
clearvars suit_time_rel suit_time_abs;

%% Extract subject weight
% The weight of the subject is the sum of forceplates forces minus the
% weight of the shoes! 

bucket.weight_from_FP = (abs(mean(forceplates.upsampled.FP1.wrenches(:,3),'omitnan')) ...
                + abs(mean(forceplates.upsampled.FP2.wrenches(:,3),'omitnan')))/9.81;
bucket.weight_shoes   = 4; %kg
bucket.weight = bucket.weight_from_FP - bucket.weight_shoes;
