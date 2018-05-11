
%--------------------------------------------------------------------------
% Xsens frequency  --> 60  Hz, acquired with MVNX2018
% shoes frequency  --> 100 Hz
% FP    frequency  --> 1K  Hz
% EMG   frequency  --> 100 Hz (to be verified)
% exoskeleton      --> passive
%--------------------------------------------------------------------------
% 5 repetitions of the same task (i.e.,5 blocks).

%% Subdivide suit.mat in 5 blocks accordingly to the masterFile
% Data in the masterFile.mat are saved in 5 separate blocks while the 
% suit.mat (extracted from MVNX) does not have this division.
tmp.block_labels = {'block1'; ...
                    'block2'; ...
                    'block3'; ...
                    'block4'; ...
                    'block5'};
tmp.nrOfBlocks = size(tmp.block_labels,1);

% 1) Extract block ranges
% From masterFile.Subject.Xsens extract the initial and final Xsens 
% timestampes for each block. Note: there is an exception for the first
% block (an error in the acquisition?)
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

% Index in meas vector corresponding to the 5 blocks
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

% 2) Subdivide suit.mat fields
suit_runtime = [];
for sensIdx = 1: size(suit.sensors,1)
    suit_runtime.sensors{sensIdx, 1}.label        = suit.sensors{sensIdx, 1}.label;
    suit_runtime.sensors{sensIdx, 1}.attachedLink = suit.sensors{sensIdx, 1}.attachedLink;
    suit_runtime.sensors{sensIdx, 1}.position     = suit.sensors{sensIdx, 1}.position;
    for blockIdx = 1 : size(tmp.block_labels,1)
        suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).block  = tmp.block_labels(blockIdx);
        suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).XsensFrameRange  = [tmp.XsensBlockRange(blockIdx).first, tmp.XsensBlockRange(blockIdx).last];
        suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).indexRange  = [tmp.blockRange(blockIdx).first, tmp.blockRange(blockIdx).last];
        
        tmp.cutRange = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
        suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).sensorOrientation = suit.sensors{sensIdx, 1}.meas.sensorOrientation(:,tmp.cutRange);
        suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).sensorFreeAcceleration = suit.sensors{sensIdx, 1}.meas.sensorFreeAcceleration(:,tmp.cutRange);
    end
end
