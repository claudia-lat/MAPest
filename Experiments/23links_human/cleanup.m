
% delete .mat file in processed_fixed (i.e., fixed folder)
delete(fullfile(bucket.pathToTask,'processed_fixed/estimation.mat'));
delete(fullfile(bucket.pathToTask,'processed_fixed/estimatedVariables.mat'));
delete(fullfile(bucket.pathToTask,'processed_fixed/y_sim.mat'));
delete(fullfile(bucket.pathToTask,'processed_fixed/y_sim_fext.mat'));
delete(fullfile(bucket.pathToTask,'processed_fixed/fixedBaseRange.mat'));

% delete .mat file in processed (i.e., floating folder)
delete(fullfile(bucket.pathToProcessedData,'estimation.mat'));
delete(fullfile(bucket.pathToProcessedData,'estimatedVariables.mat'));
delete(fullfile(bucket.pathToProcessedData,'y_sim.mat'));
delete(fullfile(bucket.pathToProcessedData,'y_sim_fext.mat'));
delete(fullfile(bucket.pathToProcessedData,'RMSE_fixedVSfloat.mat'));
delete(fullfile(bucket.pathToProcessedData,'errorVal_fixedVSfloat.mat'));