
% If the number is different we need to equalize it
if IKdata.timestamp(1) >= suit.timestamp(1)
    timestamp1 = IKdata.timestamp;
    timestamp2 = suit.timestamp;
else
    timestamp1 = suit.timestamp;
    timestamp2 = IKdata.timestamp;
end
% Define how many samples to be removed
diffOfSamples = abs(suit.nrOfFrames - IKdata.nrOfFrames);
% Decide what to cut
% -------------------------------------------------------------------------
if abs(timestamp1(1) - timestamp2(1)) <= abs(timestamp1(1) - timestamp2(2))
    % ------------------------------------
    if suit.nrOfFrames > IKdata.nrOfFrames
        % ------------------------------------
        % cut suit at the end of each signal of diffOfSamples samples
        suit.nrOfFrames = suit.nrOfFrames - diffOfSamples;
        suit.timestamp  = suit.timestamp(1:end-diffOfSamples,1);
        suit.ftShoes.Left  = suit.ftShoes.Left(:,1:end-diffOfSamples);
        suit.ftShoes.Right = suit.ftShoes.Right(:,1:end-diffOfSamples);
        % cut suit.sensors
        for sensIdx = 1 : suit.properties.nrOfSensors
            suit.sensors{sensIdx, 1}.meas.sensorFreeAcceleration  = suit.sensors{sensIdx, 1}.meas.sensorFreeAcceleration(:,1:end-diffOfSamples);
            suit.sensors{sensIdx, 1}.meas.sensorMagneticField     = suit.sensors{sensIdx, 1}.meas.sensorMagneticField(:,1:end-diffOfSamples);
            suit.sensors{sensIdx, 1}.meas.sensorOrientation       = suit.sensors{sensIdx, 1}.meas.sensorOrientation(:,1:end-diffOfSamples);
            suit.sensors{sensIdx, 1}.meas.pose.sensorPositionWRTglobal = suit.sensors{sensIdx, 1}.meas.pose.sensorPositionWRTglobal(:,1:end-diffOfSamples);
            suit.sensors{sensIdx, 1}.meas.pose.sensorOrientation       = suit.sensors{sensIdx, 1}.meas.pose.sensorOrientation(:,1:end-diffOfSamples);
            suit.sensors{sensIdx, 1}.meas.sensorPositionWRTglobal = suit.sensors{sensIdx, 1}.meas.sensorPositionWRTglobal(:,1:end-diffOfSamples);
            suit.sensors{sensIdx, 1}.meas.sensorOldAcceleration   = suit.sensors{sensIdx, 1}.meas.sensorOldAcceleration(:,1:end-diffOfSamples);
        end
        % cut suit.sensors
        for linkIdx = 1 : suit.properties.nrOfLinks
            suit.links{linkIdx, 1}.meas.orientation         = suit.links{linkIdx, 1}.meas.orientation(:,1:end-diffOfSamples);
            suit.links{linkIdx, 1}.meas.position            = suit.links{linkIdx, 1}.meas.position(:,1:end-diffOfSamples);
            suit.links{linkIdx, 1}.meas.velocity            = suit.links{linkIdx, 1}.meas.velocity(:,1:end-diffOfSamples);
            suit.links{linkIdx, 1}.meas.angularVelocity     = suit.links{linkIdx, 1}.meas.angularVelocity(:,1:end-diffOfSamples);
            suit.links{linkIdx, 1}.meas.acceleration        = suit.links{linkIdx, 1}.meas.acceleration(:,1:end-diffOfSamples);
            suit.links{linkIdx, 1}.meas.angularAcceleration = suit.links{linkIdx, 1}.meas.angularAcceleration(:,1:end-diffOfSamples);
        end
        % cut suit.joints
        for jointIdx = 1 : suit.properties.nrOfJoints
            suit.joints{jointIdx, 1}.meas.angle  = suit.joints{jointIdx, 1}.meas.angle(:,1:end-diffOfSamples);
        end
        % ------------------------------------
    else % IKdata.nrOfFrames > suit.nrOfFrames
        % ------------------------------------
        % cut IKdata at the end of each signal of diffOfSamples samples
        IKdata.nrOfFrames = IKdata.nrOfFrames - diffOfSamples;
        IKdata.timestamp  = IKdata.timestamp(1:end-diffOfSamples,1);
        % cut IK.base
        IKdata.base.quaternion      = IKdata.base.quaternion(:,1:end-diffOfSamples);
        IKdata.base.pos             = IKdata.base.pos(:,1:end-diffOfSamples);
        IKdata.base.linearVelocity  = IKdata.base.linearVelocity(:,1:end-diffOfSamples);
        IKdata.base.angularVelocity = IKdata.base.angularVelocity(:,1:end-diffOfSamples);
        % cut IKdata.joints
        for jointIdx = 1 : nrDofs
            IKdata.joints{jointIdx, 1}.angle    = IKdata.joints{jointIdx, 1}.angle(:,1:end-diffOfSamples);
            IKdata.joints{jointIdx, 1}.velocity = IKdata.joints{jointIdx, 1}.velocity(:,1:end-diffOfSamples);
        end
    end
    % -------------------------------------------------------------------------
else %abs(timestamp1(1) - timestamp2(1)) > abs(timestamp1(1) - timestamp2(2))
    % ------------------------------------
    if suit.nrOfFrames > IKdata.nrOfFrames
        % ------------------------------------
        % cut the first suit element to align timestamps
        suit.nrOfFrames = suit.nrOfFrames - diffOfSamples;
        suit.timestamp  = suit.timestamp(2:end,1);
        suit.ftShoes.Left  = suit.ftShoes.Left(:,2:end);
        suit.ftShoes.Right = suit.ftShoes.Right(:,2:end);
        % cut suit.sensors
        for sensIdx = 1 : suit.properties.nrOfSensors
            suit.sensors{sensIdx, 1}.meas.sensorFreeAcceleration  = suit.sensors{sensIdx, 1}.meas.sensorFreeAcceleration(:,2:end);
            suit.sensors{sensIdx, 1}.meas.sensorMagneticField     = suit.sensors{sensIdx, 1}.meas.sensorMagneticField(:,2:end);
            suit.sensors{sensIdx, 1}.meas.sensorOrientation       = suit.sensors{sensIdx, 1}.meas.sensorOrientation(:,2:end);
            suit.sensors{sensIdx, 1}.meas.pose.sensorPositionWRTglobal = suit.sensors{sensIdx, 1}.meas.pose.sensorPositionWRTglobal(:,2:end);
            suit.sensors{sensIdx, 1}.meas.pose.sensorOrientation       = suit.sensors{sensIdx, 1}.meas.pose.sensorOrientation(:,2:end);
            suit.sensors{sensIdx, 1}.meas.sensorPositionWRTglobal = suit.sensors{sensIdx, 1}.meas.sensorPositionWRTglobal(:,2:end);
            suit.sensors{sensIdx, 1}.meas.sensorOldAcceleration   = suit.sensors{sensIdx, 1}.meas.sensorOldAcceleration(:,2:end);
        end
        % cut suit.sensors
        for linkIdx = 1 : suit.properties.nrOfLinks
            suit.links{linkIdx, 1}.meas.orientation         = suit.links{linkIdx, 1}.meas.orientation(:,2:end);
            suit.links{linkIdx, 1}.meas.position            = suit.links{linkIdx, 1}.meas.position(:,2:end);
            suit.links{linkIdx, 1}.meas.velocity            = suit.links{linkIdx, 1}.meas.velocity(:,2:end);
            suit.links{linkIdx, 1}.meas.angularVelocity     = suit.links{linkIdx, 1}.meas.angularVelocity(:,2:end);
            suit.links{linkIdx, 1}.meas.acceleration        = suit.links{linkIdx, 1}.meas.acceleration(:,2:end);
            suit.links{linkIdx, 1}.meas.angularAcceleration = suit.links{linkIdx, 1}.meas.angularAcceleration(:,2:end);
        end
        % cut suit.joints
        for jointIdx = 1 : suit.properties.nrOfJoints
            suit.joints{jointIdx, 1}.meas.angle  = suit.joints{jointIdx, 1}.meas.angle(:,2:end);
        end
        % AND only if diffOfSamples > 1, cut final suit signals of (diffOfSamples-1)
        if diffOfSamples ~= 1
            newdiffOfSamples = diffOfSamples-1;
            suit.timestamp  = suit.timestamp(1:end-newdiffOfSamples,1);
            suit.ftShoes.Left  = suit.ftShoes.Left(:,1:end-newdiffOfSamples);
            suit.ftShoes.Right = suit.ftShoes.Right(:,1:end-newdiffOfSamples);
            % cut suit.sensors
            for sensIdx = 1 : suit.properties.nrOfSensors
                suit.sensors{sensIdx, 1}.meas.sensorFreeAcceleration  = suit.sensors{sensIdx, 1}.meas.sensorFreeAcceleration(:,1:end-newdiffOfSamples);
                suit.sensors{sensIdx, 1}.meas.sensorMagneticField     = suit.sensors{sensIdx, 1}.meas.sensorMagneticField(:,1:end-newdiffOfSamples);
                suit.sensors{sensIdx, 1}.meas.sensorOrientation       = suit.sensors{sensIdx, 1}.meas.sensorOrientation(:,1:end-newdiffOfSamples);
                suit.sensors{sensIdx, 1}.meas.pose.sensorPositionWRTglobal = suit.sensors{sensIdx, 1}.meas.pose.sensorPositionWRTglobal(:,1:end-newdiffOfSamples);
                suit.sensors{sensIdx, 1}.meas.pose.sensorOrientation       = suit.sensors{sensIdx, 1}.meas.pose.sensorOrientation(:,1:end-newdiffOfSamples);
                suit.sensors{sensIdx, 1}.meas.sensorPositionWRTglobal = suit.sensors{sensIdx, 1}.meas.sensorPositionWRTglobal(:,1:end-newdiffOfSamples);
                suit.sensors{sensIdx, 1}.meas.sensorOldAcceleration   = suit.sensors{sensIdx, 1}.meas.sensorOldAcceleration(:,1:end-newdiffOfSamples);
            end
            % cut suit.sensors
            for linkIdx = 1 : suit.properties.nrOfLinks
                suit.links{linkIdx, 1}.meas.orientation         = suit.links{linkIdx, 1}.meas.orientation(:,1:end-newdiffOfSamples);
                suit.links{linkIdx, 1}.meas.position            = suit.links{linkIdx, 1}.meas.position(:,1:end-newdiffOfSamples);
                suit.links{linkIdx, 1}.meas.velocity            = suit.links{linkIdx, 1}.meas.velocity(:,1:end-newdiffOfSamples);
                suit.links{linkIdx, 1}.meas.angularVelocity     = suit.links{linkIdx, 1}.meas.angularVelocity(:,1:end-newdiffOfSamples);
                suit.links{linkIdx, 1}.meas.acceleration        = suit.links{linkIdx, 1}.meas.acceleration(:,1:end-newdiffOfSamples);
                suit.links{linkIdx, 1}.meas.angularAcceleration = suit.links{linkIdx, 1}.meas.angularAcceleration(:,1:end-newdiffOfSamples);
            end
            % cut suit.joints
            for jointIdx = 1 : suit.properties.nrOfJoints
                suit.joints{jointIdx, 1}.meas.angle  = suit.joints{jointIdx, 1}.meas.angle(:,1:end-newdiffOfSamples);
            end
        end
        % ------------------------------------
    else % IKdata.nrOfFrames > suit.nrOfFrames
        % ------------------------------------
        % % cut the first IKdata element to align timestamps
        IKdata.nrOfFrames = IKdata.nrOfFrames - diffOfSamples;
        IKdata.timestamp  = IKdata.timestamp(2:end,1);
        % cut IK.base
        IKdata.base.quaternion      = IKdata.base.quaternion(:,2:end);
        IKdata.base.pos             = IKdata.base.pos(:,2:end);
        IKdata.base.linearVelocity  = IKdata.base.linearVelocity(:,2:end);
        IKdata.base.angularVelocity = IKdata.base.angularVelocity(:,2:end);
        % cut IKdata.joints
        for jointIdx = 1 : nrDofs
            IKdata.joints{jointIdx, 1}.angle    = IKdata.joints{jointIdx, 1}.angle(:,2:end);
            IKdata.joints{jointIdx, 1}.velocity = IKdata.joints{jointIdx, 1}.velocity(:,2:end);
        end
        % AND only if diffOfSamples > 1, cut final IKdata signals of (diffOfSamples-1)
        if diffOfSamples ~= 1
            newdiffOfSamples = diffOfSamples-1;
            IKdata.timestamp  = IKdata.timestamp(1:end-newdiffOfSamples,1);
            % cut IK.base
            IKdata.base.quaternion      = IKdata.base.quaternion(:,1:end-newdiffOfSamples);
            IKdata.base.pos             = IKdata.base.pos(:,1:end-newdiffOfSamples);
            IKdata.base.linearVelocity  = IKdata.base.linearVelocity(:,1:end-newdiffOfSamples);
            IKdata.base.angularVelocity = IKdata.base.angularVelocity(:,1:end-newdiffOfSamples);
            % cut IKdata.joints
            for jointIdx = 1 : nrDofs
                IKdata.joints{jointIdx, 1}.angle    = IKdata.joints{jointIdx, 1}.angle(:,1:end-newdiffOfSamples);
                IKdata.joints{jointIdx, 1}.velocity = IKdata.joints{jointIdx, 1}.velocity(:,1:end-newdiffOfSamples);
            end
        end
    end
end

% % % % test plot if timestamp match
% % % figure()
% % % plot(IKdata.timestamp,'o')
% % % hold on
% % % plot(suit.timestamp,'o')
% % % legend('IK','suit');
% % % title('suit vs. IKdata')
% % % ylabel('timestamps')
% % % xlabel('samples')
