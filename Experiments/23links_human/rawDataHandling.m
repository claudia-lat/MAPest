
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

block.labels = {'block1'; ...
                    'block2'; ...
                    'block3'; ...
                    'block4'; ...
                    'block5'};
block.nrOfBlocks = size(block.labels,1);

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
    for j = 1 : block.nrOfBlocks
        if suit.time.xSens(i) == tmp.XsensBlockRange(1,j).first
            tmp.blockRange(j).first = i;
        end
        if suit.time.xSens(i) == tmp.XsensBlockRange(1,j).last
            tmp.blockRange(j).last = i;
        end
    end
end

%% Timestamps struct
for blockIdx = 1 : block.nrOfBlocks
    % ---Labels
    timestampTable(blockIdx).block  = block.labels(blockIdx);

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
        timestampTable(blockIdx).masterfileTimestamps = masterFile.Subject.Xsens(blockIdx).Timestamp(tmp.cutRange,:); %exception
        timestampTable(blockIdx).masterfileTimeRT = masterFile.Subject.Xsens(blockIdx).TimeRT(tmp.cutRange,:); %exception
    else
        timestampTable(blockIdx).masterfileTimestamps  = masterFile.Subject.Xsens(blockIdx).Timestamp;
        timestampTable(blockIdx).masterfileTimeRT  = masterFile.Subject.Xsens(blockIdx).TimeRT;
    end

    % ---Cut MVNX in 5 blocks according to previous ranges
    timestampTable(blockIdx).XsensTimestampRange = [tmp.XsensBlockRange(blockIdx).first, tmp.XsensBlockRange(blockIdx).last];
    timestampTable(blockIdx).XsensCutRange = [tmp.blockRange(blockIdx).first, tmp.blockRange(blockIdx).last];
    tmp.cutRange = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
    timestampTable(blockIdx).timeMVNX = suit.time.xSens(:,tmp.cutRange);
    %timestampTable(blockIdx).timeMVNX_ms = suit.time.ms(:,tmp.cutRange);

    % ---Create a new sampling vector
    % NOTE: this vector will be used as sampling vector for the FP and
    % ftShoes data contained in the masterfile!
    tmp.RTblock_samples = size(timestampTable(blockIdx).timeMVNX,2);
    tmp.step = (timestampTable(blockIdx).masterfileTimeRT(end) - timestampTable(blockIdx).masterfileTimeRT(1))/(tmp.RTblock_samples -1);
    timestampTable(blockIdx).masterfileNewTimeRT = timestampTable(blockIdx).masterfileTimeRT(1) : tmp.step : timestampTable(blockIdx).masterfileTimeRT(end);
end

%% Subdivide suit.mat meas in 5 blocks accordingly to the above division

tmp.cutRange = cell(5,1);
for sensIdx = 1: size(suit.sensors,1)
    suit_runtime.sensors{sensIdx, 1}.label        = suit.sensors{sensIdx, 1}.label;
    suit_runtime.sensors{sensIdx, 1}.attachedLink = suit.sensors{sensIdx, 1}.attachedLink;
    suit_runtime.sensors{sensIdx, 1}.position     = suit.sensors{sensIdx, 1}.position;

    for blockIdx = 1 : block.nrOfBlocks
        % ---Labels
        suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).block  = block.labels(blockIdx);
        % ---Cut (useful) meas
        tmp.cutRange{blockIdx} = (tmp.blockRange(blockIdx).first : tmp.blockRange(blockIdx).last);
        suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).sensorOrientation = suit.sensors{sensIdx, 1}.meas.sensorOrientation(:,tmp.cutRange{blockIdx});
        suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).sensorFreeAcceleration = suit.sensors{sensIdx, 1}.meas.sensorFreeAcceleration(:,tmp.cutRange{blockIdx});
        % NOTE: MVNX data do not need interpolation!
    end
end

%% Transform the sensorFreeAcceleration of MVNX2018 into the oldest version
if ~exist(fullfile(bucket.pathToProcessedData,'suit_runtime.mat'))
    quaternion = iDynTree.Vector4();
    G_R_S = iDynTree.Rotation();
    gravity = [0; 0; -9.81];
    for sensIdx = 1: size(suit.sensors,1)
        for blockIdx = 1 : block.nrOfBlocks
            len = size(suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).sensorOrientation,2);
            for lenIdx = 1 : len
                quaternion.fromMatlab(suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).sensorOrientation(:,lenIdx));
                G_R_S.fromQuaternion(quaternion);
                % Transformation:        S_a_old = S_R_G * (G_a_new - gravity)
                suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).sensorOldAcceleration(:,lenIdx) = ...
                    (G_R_S.toMatlab())' * (suit_runtime.sensors{sensIdx, 1}.meas(blockIdx).sensorFreeAcceleration(:,lenIdx) - gravity);
            end
        end
    end
    save(fullfile(bucket.pathToProcessedData,'/suit_runtime.mat'),'suit_runtime');
else
    load(fullfile(bucket.pathToProcessedData,'/suit_runtime.mat'),'suit_runtime');
end

%% Create the synchroData struct
% Struct where every external is synchronized:
% - masterTime
% - ftshoes
% - state (q,dq)
% - ddq

for blockIdx = 1 : block.nrOfBlocks
    synchroData(blockIdx).block = block.labels(blockIdx);
    synchroData(blockIdx).masterTime = timestampTable(blockIdx).masterfileNewTimeRT;
    synchroData(blockIdx).q   = human_state_tmp.q(:,tmp.cutRange{blockIdx});
    synchroData(blockIdx).dq  = human_state_tmp.dq(:,tmp.cutRange{blockIdx});
    synchroData(blockIdx).ddq = human_ddq_tmp(:,tmp.cutRange{blockIdx});
end
clearvars human_state_tmp human_ddq_tmp;

%% ftShoes Interpolation
% FS1 --> Right ftShoe wrench
% FS2 --> Left ftShoe wrench

% Cutting (when needed) signals
for blockIdx = 1 : block.nrOfBlocks
     tmp.length = size(masterFile.Subject.FS(blockIdx).Measurement,1);
     for j = 1 : tmp.length
          if (timestampTable(blockIdx).masterfileNewTimeRT(1) - masterFile.Subject.FS(blockIdx).TimeRT(j) < 0.01)
              tmp.idx(blockIdx) = j;
              break;
          end
     end
     tmp.ftShoes.cutRange = (tmp.idx(blockIdx) : size(masterFile.Subject.FS(blockIdx).Measurement,1));
     if tmp.ftShoes.cutRange(1) ~= 1
        tmp.ftShoes.cut(blockIdx).RightShoe = masterFile.Subject.FS(blockIdx).FS1(tmp.ftShoes.cutRange,:);
        tmp.ftShoes.cut(blockIdx).LeftShoe  = masterFile.Subject.FS(blockIdx).FS2(tmp.ftShoes.cutRange,:);
        tmp.ftShoes.cut(blockIdx).timeRT    = masterFile.Subject.FS(blockIdx).TimeRT(tmp.ftShoes.cutRange,:);
     else
        tmp.ftShoes.cut(blockIdx).RightShoe = masterFile.Subject.FS(blockIdx).FS1;
        tmp.ftShoes.cut(blockIdx).LeftShoe  = masterFile.Subject.FS(blockIdx).FS2;
        tmp.ftShoes.cut(blockIdx).timeRT    = masterFile.Subject.FS(blockIdx).TimeRT;
     end
end

% Interpolation
% SF = sensor frame
for blockIdx = 1 : block.nrOfBlocks
    for i = 1 : 6
        synchroData(blockIdx).RightShoe_SF(:,i) = interp1(tmp.ftShoes.cut(blockIdx).timeRT, ...
                                                 tmp.ftShoes.cut(blockIdx).RightShoe(:,i), ...
                                                 timestampTable(blockIdx).masterfileNewTimeRT);
        synchroData(blockIdx).LeftShoe_SF(:,i)  = interp1(tmp.ftShoes.cut(blockIdx).timeRT, ...
                                                 tmp.ftShoes.cut(blockIdx).LeftShoe(:,i), ...
                                                 timestampTable(blockIdx).masterfileNewTimeRT);
    end
end

%% forcePlates Interpolation
% to be done

%% Cleaning up workspace
clearvars tmp timestampTable;
