function [ parsed_forces ] = parseYARPftShoes_fromDriver( filename_dumpForces)
%PARSEYARPFTSHOES_FROMDRIVER parses YARP forces dumped from the ftShoes
% YARP driver.  At this stage, forces are in ftShoes reference frames.
%
% Input: 
% - filename_dumpForces: name of the file.log
% - numberOfForces: number of forces involved in the analysis. For example,
%                   it is 2 if there are only 2 forceplates, it is 4 if 
%                   there are a combination of 2 forceplates and 2 robot
%                   contacts.             
% Output:
% - parsed_forces: parsed ftShoes forces in a Matlab struct.
%
% The parser is built considering the following data structure presnet in
% the .log file:
%
% [iteration number  timestamp  fx  fy  fz  mx  my  mz]
%
% whose formatSpec is [%d  %f  %f  %f  %f  %f  %f  %f].


%% Access to the file
fileID = fopen(filename_dumpForces);

formatSpec = '%d %f ';
formatSpec = [formatSpec, ''];
for k = 1 : 6
    formatSpec = [formatSpec, '%f '];
end
formatSpec = [formatSpec, ''];

% C contains parsed data from the file according to the formatSpec
C = textscan(fileID, formatSpec,'MultipleDelimsAsOne', 1, 'Delimiter', {' ', '\b'});

fclose(fileID);

%% Map C to a proper structure (the Matlab struct that we want as output)

% Number of samples
numOfSamples = length(C{1});
% Time
parsed_forces.time = C{2}';
% Time normalized to zero
parsed_forces.timeNormToZero = parsed_forces.time - repmat(parsed_forces.time(1), size(parsed_forces.time)); 

% Vector of wrenches (3 forces + 3 vectors)
wrenches = zeros(6, numOfSamples);
for j = 1:6
    C_force= C{j+2};
    for i = 1:numOfSamples
        wrenches(j,i) = C_force(i,:);
    end
end

parsed_forces.forces  = wrenches(1:3,:);
parsed_forces.moments = wrenches(4:6,:);

end


