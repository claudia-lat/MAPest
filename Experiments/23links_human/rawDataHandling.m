
%--------------------------------------------------------------------------
% Xsens frequency  --> 60  Hz, acquired with MVNX2018
% shoes frequency  --> 100 Hz
% FP    frequency  --> 1K  Hz
% EMG   frequency  --> 100 Hz (to be verified)
% exoskeleton      --> passive
%--------------------------------------------------------------------------
% 5 repetitions of the same task (i.e.,5 blocks).
% Data in the masterFile.mat are saved in 5 separate blocks while the 
% suit.mat (extracted from MVNX) does not have this division.

tmp.block_labels = {'block1'; ...
                    'block2'; ...
                    'block3'; ...
                    'block4'; ...
                    'block5'};
tmp.nrOfBlocks = size(tmp.block_labels,1);

for i = 1 : length(masterFile.Subject.Xsens(1).Timestamp)
    tmp.block1 = masterFile.Subject.Xsens(1).Timestamp - masterFile.Subject.Xsens(1).Timestamp(end);
    if tmp.block1(i)<=0
        tmp.block1Init = i;
        break
    end
end
for i = 1 : length(masterFile.Subject.Xsens) %5 blocks
    if i == 1
        tmp.XsensBlockRange(i).first = masterFile.Subject.Xsens(i).Timestamp(tmp.block1Init);
    else
        tmp.XsensBlockRange(i).first = masterFile.Subject.Xsens(i).Timestamp(1);
    end
    tmp.XsensBlockRange(i).last = masterFile.Subject.Xsens(i).Timestamp(end);
end

for i = 1 : size(suit.sensors{1, 1}.meas.sensorOrientation,2) % sens1 since it is equal for all the sensors
    for j = 1 : tmp.nrOfBlocks
        if suit.time.xSens(i) == tmp.XsensBlockRange(1,j).first
            tmp.blockRange(j).first = i;
        end
        if suit.time.xSens(i) == tmp.XsensBlockRange(1,j).last
            tmp.blockRange(j).last = i;
        end
    end
end

%% Subdivide suit.mat in 5 blocks accordingly to the masterFile
% No synchronization among the different sensors is adopted at this stage!
for sensIdx = 1: size(suit.sensors,1)
    suit_runtime.sensors{sensIdx, 1}.label        = suit.sensors{sensIdx, 1}.label;
    suit_runtime.sensors{sensIdx, 1}.attachedLink = suit.sensors{sensIdx, 1}.attachedLink;
    suit_runtime.sensors{sensIdx, 1}.position     = suit.sensors{sensIdx, 1}.position;

    for blockIdx = 1 : (tmp.nrOfBlocks)
        % ---Labels
        suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).block  = tmp.block_labels(blockIdx);
        
        % ---Xsens Timestamp Range
        if blockIdx == 1 %exception
            for i = 1: size(masterFile.Subject.Xsens(blockIdx).Timestamp,1)
                if masterFile.Subject.Xsens(blockIdx).Timestamp(i) == tmp.XsensBlockRange(1).first
                    tmp.exception_first = i;
                end
                if masterFile.Subject.Xsens(blockIdx).Timestamp(i) == tmp.XsensBlockRange(1).last
                    tmp.exception_last = i;
                end
            end
            tmp.cutRange = (tmp.exception_first : tmp.exception_last);
            suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).masterfileTimestamps = masterFile.Subject.Xsens(blockIdx).Timestamp(tmp.cutRange,:); %exception
        else
            suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).masterfileTimestamps  = masterFile.Subject.Xsens(blockIdx).Timestamp;
        end

        % ---Cut (useful) meas in 5 blocks
        suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).XsensTimestampRange  = [tmp.XsensBlockRange(blockIdx).first, tmp.XsensBlockRange(blockIdx).last];
        tmp.cutRange = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
        suit_tmp.sensors{sensIdx, 1}.meas(blockIdx).sensorOrientation = suit.sensors{sensIdx, 1}.meas.sensorOrientation(:,tmp.cutRange);
        suit_tmp.sensors{sensIdx, 1}.meas(blockIdx).sensorFreeAcceleration = suit.sensors{sensIdx, 1}.meas.sensorFreeAcceleration(:,tmp.cutRange);

        % ---Interpolation:
        % Blocks from masterFile and suit do not have the same lenght!! We need to
        % interpolate (useful) meas according to the masterFile (master) timestamps.
        tmp.slaveTime  = suit.time.xSens(:,tmp.cutRange)';
        if blockIdx == 1
            tmp.masterTime = suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).masterfileTimestamps; %exception
        else
            tmp.masterTime = masterFile.Subject.Xsens(blockIdx).Timestamp;
        end
        for i = 1 : 4 %column elements in the orientation vector
            suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).sensorOrientation(i,:) = interp1(tmp.slaveTime, ...
                    suit_tmp.sensors{sensIdx, 1}.meas(blockIdx).sensorOrientation(i,:), tmp.masterTime);
        end
        for i = 1 : 3 %column elements in the freAcceleration vector
            suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).sensorFreeAcceleration(i,:) = interp1(tmp.slaveTime, ...
                    suit_tmp.sensors{sensIdx, 1}.meas(blockIdx).sensorFreeAcceleration(i,:), tmp.masterTime);
        end
    end
end

%% Cleaning up workspace
clearvars suit_tmp;
