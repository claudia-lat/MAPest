function [sideHandlerSync] = synchroIndividualShoe(sideHandler)
%SYNCHROSHOES synchronizes the Front, Rear and Total YARP acquisition 
% within a single shoe.

format long

timeFront = sideHandler.frontForce.time;
timeRear  = sideHandler.rearForce.time;
timeTotal = sideHandler.totalForce.time;

% --------------------------------
% Cut the first part of the signal
% --------------------------------
% Find max among each first timestamp 
maxArray = [timeFront(1), timeRear(1), timeTotal(1)];
maxFirstTimeStamp = max(maxArray);

% Check maxFirstTimeStamp in FRONT
for i = 1: length(timeFront)-1
    front_diff(i) = timeFront(:,i+1) - timeFront(:,i);  
end
meanFront = mean(front_diff);
for i = 1 : size(timeFront,2)
   if timeFront(i) == maxFirstTimeStamp || abs(timeFront(i) - maxFirstTimeStamp)< mean(front_diff)
       indexFront = i;
       break
   end
end
% Cut signals
timeFront    = timeFront(:,indexFront:end);
forcesFront  = sideHandler.frontForce.forces(:,indexFront:end);
momentsFront = sideHandler.frontForce.moments(:,indexFront:end);

% Check maxFirstTimeStamp in REAR
for i = 1: length(timeRear)-1
    rear_diff(i) = timeRear(:,i+1) - timeRear(:,i);  
end
meanRear = mean(rear_diff);
for i = 1 : size(timeRear,2)
   if timeRear(i) == maxFirstTimeStamp || abs(timeRear(i) - maxFirstTimeStamp)< mean(rear_diff)
       indexRear = i;
       break
   end
end
% Cut signals
timeRear = timeRear(:,indexRear:end);
forcesRear  = sideHandler.rearForce.forces(:,indexRear:end);
momentsRear = sideHandler.rearForce.moments(:,indexRear:end);

% Check maxFirstTimeStamp in TOTAL
for i = 1: length(timeTotal)-1
    total_diff(i) = timeTotal(:,i+1) - timeTotal(:,i);  
end
meanTotal = mean(total_diff);
for i = 1 : size(timeTotal,2)
   if timeTotal(i) == maxFirstTimeStamp || abs(timeTotal(i) - maxFirstTimeStamp)< mean(total_diff)
       indexTotal = i;
       break
   end
end
% Cut signal
timeTotal = timeTotal(:,indexTotal:end);
forcesTotal  = sideHandler.totalForce.forces(:,indexTotal:end);
momentsTotal = sideHandler.totalForce.moments(:,indexTotal:end);

% --------------------------------
% Cut the final part of the signal
% --------------------------------
% Find min among the length of each cut timestamp 
minArray = [length(timeFront), length(timeRear), length(timeTotal)];
minLengthTimeStamp = min(minArray);

% Cut all the signals according to minLengthTimeStamp
sideHandlerSync.frontForce.time    = timeFront(:,1:minLengthTimeStamp);
sideHandlerSync.frontForce.forces  = forcesFront(:,1:minLengthTimeStamp);
sideHandlerSync.frontForce.moments = momentsFront(:,1:minLengthTimeStamp);

sideHandlerSync.rearForce.time    = timeRear(:,1:minLengthTimeStamp);
sideHandlerSync.rearForce.forces  = forcesRear(:,1:minLengthTimeStamp);
sideHandlerSync.rearForce.moments = momentsRear(:,1:minLengthTimeStamp);

sideHandlerSync.totalForce.time    = timeTotal(:,1:minLengthTimeStamp);
sideHandlerSync.totalForce.forces  = forcesTotal(:,1:minLengthTimeStamp);
sideHandlerSync.totalForce.moments = momentsTotal(:,1:minLengthTimeStamp);

end

