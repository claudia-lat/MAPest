
%% Compute or load comaparison params
addpath(genpath('test'));
for blockIdx = 1 : block.nrOfBlocks
    tau_iDyn(blockIdx).block = block.labels(blockIdx);
end
if ~exist(fullfile(bucket.pathToProcessedData,'IDcomparisonParams.mat'), 'file')
    test_IDcomparisonParamsCollection;
else
    load(fullfile(bucket.pathToProcessedData,'IDcomparisonParams.mat'))
end

%% Benchmark with iDynTree::kinDynComputation::InverseDynamics()
if opts.iDynID_kinDynClass
   for blockIdx = 1 : block.nrOfBlocks
       zeros6 = zeros(6,length(synchroData(blockIdx).masterTime));
       zeros3 = zeros(3,length(synchroData(blockIdx).masterTime));

        disp('-------------------------------------------------------------------');
        disp(strcat('[Start] iDynTree ID (via kinDyn) computation for Block ',num2str(blockIdx),'...'));
        tau_iDyn(blockIdx).tau_kinDyn = iDynTreeID_kinDyn_floating(human_kinDynComp, ...
            currentBase, ...
            IDcomparisonParams.orientation(blockIdx).baseOrientation, ...
            IDcomparisonParams.basePosition(blockIdx).basePos_wrtG, ...
            IDcomparisonParams.baseVel(blockIdx).baseVel_wrtG, ...
            IDcomparisonParams.baseAcc(blockIdx).baseAcc_wrtG, ...
            synchroKin(blockIdx), ...
            shoesZeros(blockIdx)); %shoesZeros(blockIdx));

        disp(strcat('[End] iDynTree ID (via kinDyn) computation for Block ',num2str(blockIdx)));
    end
    save(fullfile(bucket.pathToProcessedData,'tau_iDyn.mat'),'tau_iDyn');
end

%% Benchmark with iDynTree::ExtWrenchesAndJointTorquesEstimator::estimateExtWrenchesAndJointTorques()	
if opts.iDynID_estimClass
   for blockIdx = 1 : block.nrOfBlocks
        zeros3 = zeros(3,length(synchroData(blockIdx).masterTime));
        IDcomparisonParams.baseAcc(blockIdx).baseProperAcc_wrtBase = zeros3;
        IDcomparisonParams.baseAcc(blockIdx).baseAngAcc_wrtBase = zeros3;

        disp('-------------------------------------------------------------------');
        disp(strcat('[Start] iDynTree tau estimation for Block ',num2str(blockIdx),'...'));
        tau_iDyn(blockIdx).tau_estim = iDynTreeID_estimClass_floating(bucket.filenameURDF, ...
            cell2iDynTreeStringVector(selectedJoints), ...
            currentBase, ...
            synchroKin(blockIdx), ...
            baseAngVel(blockIdx).baseAngVelocity, ... % IDcomparisonParams.baseVel(blockIdx).baseAngVel_wrtBase, ...
            IDcomparisonParams.baseAcc(blockIdx), ...
            shoes(blockIdx)); %shoesZeros(blockIdx));
        disp(strcat('[End] iDynTree torque and forces estimation for Block ',num2str(blockIdx)));
   end
   save(fullfile(bucket.pathToProcessedData,'tau_iDyn_estim.mat'),'tau_iDyn');
end

%% Benchmark with OpenSim ID
% if opts.OsimID
%   % TO DO
% end
