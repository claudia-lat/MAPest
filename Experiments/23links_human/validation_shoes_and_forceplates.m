%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  THIS SCRIPT IS FOR THE SENSOR COMBINATION ftShoes + Forceplates  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The script allows to validate shoes and forceplates.  The suit struct is
% not required for this comparison.

%The signals situation is the following:
% SUIT         |-----|------------------|-------|   240Hz, unix xsens time
% FP Upsampled |-----|------------------|-------|   240Hz, abs cortex time
% SHOES              |------------------|           100Hz, unix yarp time


%% Cut forceplates with suit rangeCut
comparison.forceplates.cut.time = comparison.forceplates.upsampled.time(comparison.rangeCut);
comparison.forceplates.cut.FP1.wrenches = comparison.forceplates.upsampled.FP1.wrenches(comparison.rangeCut,:);
comparison.forceplates.cut.FP2.wrenches = comparison.forceplates.upsampled.FP2.wrenches(comparison.rangeCut,:);
% we know at the current stage that suit and forceplates are synchro!

%% Downsample forceplates to shoes

% transform rel time into abs time
shoes_time_abs = zeros(size(comparison.shoes.time));
for i = 1 : size(comparison.shoes.time,2)
    shoes_time_abs(:,i) = comparison.shoes.time(:,i) - comparison.shoes.time(:,1);
end

fp_time_abs = zeros(size(comparison.forceplates.cut.time));
for i = 1 : size(comparison.forceplates.cut.time,2)
    fp_time_abs(:,i) = comparison.forceplates.cut.time(:,i) - comparison.forceplates.cut.time(:,1);
end

comparison.masterTime = shoes_time_abs; % abs time
comparison.slaveTime = fp_time_abs; % abs time

for i = 1 : size(comparison.forceplates.cut.FP1.wrenches,2)
    comparison.forceplates.downsampled.FP1.wrenches(:,i) = interp1(comparison.slaveTime, ...
                                         comparison.forceplates.cut.FP1.wrenches(:,i), ...
                                         comparison.masterTime);
    comparison.forceplates.downsampled.FP2.wrenches(:,i) = interp1(comparison.slaveTime, ...
                                         comparison.forceplates.cut.FP2.wrenches(:,i), ...
                                         comparison.masterTime);
end
