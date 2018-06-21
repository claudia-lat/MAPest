
% Script to handle the info of the angular velocity of the base.
% This value is mandatorily required in the floating-base formalism.
% The new MVNX2018 doesn't provide the sensorAngularVelocity anymore. 

if (currentBase == 'LeftFoot' | currentBase == 'RightFoot')
    % If the base is one foot since we know that the foot is fixed during 
    % the task , we can assume that its angular velocity is equal to zero.
    bucket.baseAngVel = [0 0 0];
end