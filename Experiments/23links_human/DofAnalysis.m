
%% DoF analysis specification

% Change the name of T4Shoulder in C7Shoulder
for jointsIdx = 1 : suit.properties.nrOfJoints
    if strcmp(suit.joints{jointsIdx, 1}.label,'jLeftT4Shoulder')
        suit.joints{jointsIdx, 1}.label = 'jLeftC7Shoulder';
    end
    if strcmp(suit.joints{jointsIdx, 1}.label,'jRightT4Shoulder')
        suit.joints{jointsIdx, 1}.label = 'jRightC7Shoulder';
    end
end

% Model with 48 dofs
if opts.analysis_48dofURDF
    nrDofs = 48;
    selectedJoints = {'jL5S1_rotx';'jL5S1_roty'; ...
        'jL4L3_rotx';'jL4L3_roty'; ...
        'jL1T12_rotx';'jL1T12_roty'; ...
        'jT9T8_rotx';'jT9T8_roty';'jT9T8_rotz'; ...
        'jT1C7_rotx';'jT1C7_roty';'jT1C7_rotz'; ...
        'jC1Head_rotx';'jC1Head_roty'; ...
        'jRightC7Shoulder_rotx'; ...
        'jRightShoulder_rotx';'jRightShoulder_roty';'jRightShoulder_rotz'; ...
        'jRightElbow_roty';'jRightElbow_rotz'; ...
        'jRightWrist_rotx';'jRightWrist_rotz'; ...
        'jLeftC7Shoulder_rotx'; ...
        'jLeftShoulder_rotx';'jLeftShoulder_roty';'jLeftShoulder_rotz'; ...
        'jLeftElbow_roty';'jLeftElbow_rotz'; ...
        'jLeftWrist_rotx';'jLeftWrist_rotz'; ...
        'jRightHip_rotx';'jRightHip_roty';'jRightHip_rotz'; ...
        'jRightKnee_roty';'jRightKnee_rotz'; ...
        'jRightAnkle_rotx';'jRightAnkle_roty';'jRightAnkle_rotz'; ...
        'jRightBallFoot_roty'; ...
        'jLeftHip_rotx';'jLeftHip_roty';'jLeftHip_rotz'; ...
        'jLeftKnee_roty';'jLeftKnee_rotz'; ...
        'jLeftAnkle_rotx';'jLeftAnkle_roty';'jLeftAnkle_rotz'; ...
        'jLeftBallFoot_roty'};
end

% Model with 66 dofs
if opts.analysis_66dofURDF
    nrDofs = 66;
    selectedJoints = {'jL5S1_rotx';'jL5S1_roty'; 'jL5S1_rotz'; ...
        'jL4L3_rotx';'jL4L3_roty'; 'jL4L3_rotz'; ...
        'jL1T12_rotx';'jL1T12_roty'; 'jL1T12_rotz'; ...
        'jT9T8_rotx';'jT9T8_roty';'jT9T8_rotz'; ...
        'jT1C7_rotx';'jT1C7_roty';'jT1C7_rotz'; ...
        'jC1Head_rotx';'jC1Head_roty'; 'jC1Head_rotz'; ...
        'jRightC7Shoulder_rotx'; 'jRightC7Shoulder_roty';'jRightC7Shoulder_rotz'; ...
        'jRightShoulder_rotx';'jRightShoulder_roty';'jRightShoulder_rotz'; ...
        'jRightElbow_rotx';'jRightElbow_roty';'jRightElbow_rotz'; ...
        'jRightWrist_rotx';'jRightWrist_roty';'jRightWrist_rotz'; ...
        'jLeftC7Shoulder_rotx';'jLeftC7Shoulder_roty';'jLeftC7Shoulder_rotz'; ...
        'jLeftShoulder_rotx';'jLeftShoulder_roty';'jLeftShoulder_rotz'; ...
        'jLeftElbow_rotx';'jLeftElbow_roty';'jLeftElbow_rotz'; ...
        'jLeftWrist_rotx';'jLeftWrist_roty';'jLeftWrist_rotz'; ...
        'jRightHip_rotx';'jRightHip_roty';'jRightHip_rotz'; ...
        'jRightKnee_rotx';'jRightKnee_roty';'jRightKnee_rotz'; ...
        'jRightAnkle_rotx';'jRightAnkle_roty';'jRightAnkle_rotz'; ...
        'jRightBallFoot_rotx';'jRightBallFoot_roty';'jRightBallFoot_rotz'; ...
        'jLeftHip_rotx';'jLeftHip_roty';'jLeftHip_rotz'; ...
        'jLeftKnee_rotx';'jLeftKnee_roty';'jLeftKnee_rotz'; ...
        'jLeftAnkle_rotx';'jLeftAnkle_roty';'jLeftAnkle_rotz'; ...
        'jLeftBallFoot_rotx'; 'jLeftBallFoot_roty'; 'jLeftBallFoot_rotz'};
end