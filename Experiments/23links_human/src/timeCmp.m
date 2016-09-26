function [slaveIndex, masterIndex] = timeCmp(masterTime, slaveTime, threshold)
% TIMECMP: Function to synchronize two system with different sampling

% Input:
% - masterTime : time data of the system that impose the sampling;
% - slaveTime : time data of the system that have to be synchronized with the master system;
% - threshold : [ms] maximum difference between two time values that make the
%               synchronization acceptable;
% Output:
% - slaveIndex : index for the sample of the slave that have a corresponding master data, 
%                zero if there is not a slave sample that corresponds to the master;
% - masterIndex : index for the sample of the master that have a corresponding slave data.

lenMaster = length(masterTime);
lenSlave = length(slaveTime);
slaveIndex = zeros(lenMaster, 1);
masterIndex = [];
for i=1:lenMaster
    for j=1:lenSlave
        if (slaveIndex(i)==0)
            if (isequal(round(masterTime(i)),round(slaveTime(j)))==1)
                slaveIndex(i)=j;
                masterIndex = [masterIndex; i];
            else          
                [value,index] = min(abs(round(masterTime(i))-round(slaveTime)));
                if value <= threshold
                    slaveIndex(i) = index;
                    masterIndex = [masterIndex; i];
                end     
            end
        end
    end
end
end