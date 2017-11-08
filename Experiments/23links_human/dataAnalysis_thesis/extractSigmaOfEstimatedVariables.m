
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIGMA TAU
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
range_cut_plot = (1:2000); %2000 is manually added in MAP computation

%% Extract range from dynamics variables
% dofs: 3
range_rightAnkle_rotx = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_rotx'));
range_rightAnkle_roty = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_roty'));
range_rightAnkle_rotz = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightAnkle_rotz'));
range_leftAnkle_rotx  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_rotx'));
range_leftAnkle_roty  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_roty'));
range_leftAnkle_rotz  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftAnkle_rotz'));
% dofs: 2
range_rightKnee_roty = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightKnee_roty'));
range_rightKnee_rotz = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightKnee_rotz'));
range_leftKnee_roty  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftKnee_roty'));
range_leftKnee_rotz  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftKnee_rotz'));
% dofs: 3
range_rightHip_rotx = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_rotx'));
range_rightHip_roty = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_roty'));
range_rightHip_rotz = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightHip_rotz'));
range_leftHip_rotx  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftHip_rotx'));
range_leftHip_roty  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftHip_roty'));
range_leftHip_rotz  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftHip_rotz'));
% dofs: 2
range_L5S1_rotx = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jL5S1_rotx'));
range_L5S1_roty = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jL5S1_roty'));
% dofs: 2
range_L4L3_rotx = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jL4L3_rotx'));
range_L4L3_roty = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jL4L3_roty'));
% dofs: 2
range_L1T12_rotx = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jL1T12_rotx'));
range_L1T12_roty = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jL1T12_roty'));
% dofs:  3
range_T9T8_rotx = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jT9T8_rotx'));
range_T9T8_roty = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jT9T8_roty'));
range_T9T8_rotz = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jT9T8_rotz'));
% dofs:  3
range_T1C7_rotx = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jT1C7_rotx'));
range_T1C7_roty = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jT1C7_roty'));
range_T1C7_rotz = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jT1C7_rotz'));
% dofs: 2
range_C1Head_rotx = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jC1Head_rotx'));
range_C1Head_roty = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jC1Head_roty'));
% dofs: 1
range_rightC7Shoulder_rotx = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightC7Shoulder_rotx'));
range_leftC7Shoulder_rotx  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftC7Shoulder_rotx'));
% dofs: 3
range_rightShoulder_rotx = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_rotx'));
range_rightShoulder_roty = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_roty'));
range_rightShoulder_rotz = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightShoulder_rotz'));
range_leftShoulder_rotx  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftShoulder_rotx'));
range_leftShoulder_roty  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftShoulder_roty'));
range_leftShoulder_rotz  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftShoulder_rotz'));
% dofs: 2
range_rightElbow_roty = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightElbow_roty'));
range_rightElbow_rotz = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightElbow_rotz'));
range_leftElbow_roty  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftElbow_roty'));
range_leftElbow_rotz  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftElbow_rotz'));
% dofs: 2
range_rightWrist_rotx = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightWrist_rotx'));
range_rightWrist_rotz = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jRightWrist_rotz'));
range_leftWrist_rotx  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftWrist_rotx'));
range_leftWrist_rotz  = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, 'jLeftWrist_rotz'));

%% Fill Sigma_tau struct
Sigma_tau = struct;
for i = 1: size(range_cut_plot,2)
    Sigma_tau.rightAnkle_rotx(:,i) = Sigma_dgiveny{i}(range_rightAnkle_rotx,range_rightAnkle_rotx);
    Sigma_tau.rightAnkle_roty(:,i) = Sigma_dgiveny{i}(range_rightAnkle_roty,range_rightAnkle_roty);
    Sigma_tau.rightAnkle_rotz(:,i) = Sigma_dgiveny{i}(range_rightAnkle_rotz,range_rightAnkle_rotz);
    Sigma_tau.leftAnkle_rotx(:,i)  = Sigma_dgiveny{i}(range_leftAnkle_rotx,range_leftAnkle_rotx);
    Sigma_tau.leftAnkle_roty(:,i)  = Sigma_dgiveny{i}(range_leftAnkle_roty,range_leftAnkle_roty);
    Sigma_tau.leftAnkle_rotz(:,i)  = Sigma_dgiveny{i}(range_leftAnkle_rotz,range_leftAnkle_rotz);
    %
    Sigma_tau.rightKnee_roty(:,i) = Sigma_dgiveny{i}(range_rightKnee_roty,range_rightKnee_roty);
    Sigma_tau.rightKnee_rotz(:,i) = Sigma_dgiveny{i}(range_rightKnee_rotz,range_rightKnee_rotz);
    Sigma_tau.leftKnee_roty(:,i)  = Sigma_dgiveny{i}(range_leftKnee_roty,range_leftKnee_roty);
    Sigma_tau.leftKnee_rotz(:,i)  = Sigma_dgiveny{i}(range_leftKnee_rotz,range_leftKnee_rotz);
    %
    Sigma_tau.rightHip_rotx(:,i) = Sigma_dgiveny{i}(range_rightHip_rotx,range_rightHip_rotx);
    Sigma_tau.rightHip_roty(:,i) = Sigma_dgiveny{i}(range_rightHip_roty,range_rightHip_roty);
    Sigma_tau.rightHip_rotz(:,i) = Sigma_dgiveny{i}(range_rightHip_rotz,range_rightHip_rotz);
    Sigma_tau.leftHip_rotx(:,i)  = Sigma_dgiveny{i}(range_leftHip_rotx,range_leftHip_rotx);
    Sigma_tau.leftHip_roty(:,i)  = Sigma_dgiveny{i}(range_leftHip_roty,range_leftHip_roty);
    Sigma_tau.leftHip_rotz(:,i)  = Sigma_dgiveny{i}(range_leftHip_rotz,range_leftHip_rotz);
    %
    Sigma_tau.L5S1_rotx(:,i) = Sigma_dgiveny{i}(range_L5S1_rotx,range_L5S1_rotx);
    Sigma_tau.L5S1_roty(:,i) = Sigma_dgiveny{i}(range_L5S1_roty,range_L5S1_roty);
    %
    Sigma_tau.L4L3_rotx(:,i) = Sigma_dgiveny{i}(range_L4L3_rotx,range_L4L3_rotx);
    Sigma_tau.L4L3_roty(:,i) = Sigma_dgiveny{i}(range_L4L3_roty,range_L4L3_roty);
    %
    Sigma_tau.L1T12_rotx(:,i) = Sigma_dgiveny{i}(range_L1T12_rotx,range_L1T12_rotx);
    Sigma_tau.L1T12_roty(:,i) = Sigma_dgiveny{i}(range_L1T12_roty,range_L1T12_roty);
    %
    Sigma_tau.T1C7_rotx(:,i) = Sigma_dgiveny{i}(range_T1C7_rotx,range_T1C7_rotx);
    Sigma_tau.T1C7_roty(:,i) = Sigma_dgiveny{i}(range_T1C7_roty,range_T1C7_roty);
    Sigma_tau.T1C7_rotz(:,i) = Sigma_dgiveny{i}(range_T1C7_rotz,range_T1C7_rotz);
    %
    Sigma_tau.C1Head_rotx(:,i) = Sigma_dgiveny{i}(range_C1Head_rotx,range_C1Head_rotx);
    Sigma_tau.C1Head_roty(:,i) = Sigma_dgiveny{i}(range_C1Head_roty,range_C1Head_roty);
    %
    Sigma_tau.rightC7Shoulder_rotx(:,i) = Sigma_dgiveny{i}(range_rightC7Shoulder_rotx,range_rightC7Shoulder_rotx);
    Sigma_tau.leftC7Shoulder_rotx(:,i)  = Sigma_dgiveny{i}(range_leftC7Shoulder_rotx,range_leftC7Shoulder_rotx);
    %
    Sigma_tau.rightShoulder_rotx(:,i) = Sigma_dgiveny{i}(range_rightShoulder_rotx,range_rightShoulder_rotx);
    Sigma_tau.rightShoulder_roty(:,i) = Sigma_dgiveny{i}(range_rightShoulder_roty,range_rightShoulder_roty);
    Sigma_tau.rightShoulder_rotz(:,i) = Sigma_dgiveny{i}(range_rightShoulder_rotz,range_rightShoulder_rotz);
    Sigma_tau.leftShoulder_rotx(:,i)  = Sigma_dgiveny{i}(range_leftShoulder_rotx,range_leftShoulder_rotx);
    Sigma_tau.leftShoulder_roty(:,i)  = Sigma_dgiveny{i}(range_leftShoulder_roty,range_leftShoulder_roty);
    Sigma_tau.leftShoulder_rotz(:,i)  = Sigma_dgiveny{i}(range_leftShoulder_rotz,range_leftShoulder_rotz);
    %
    Sigma_tau.rightElbow_roty(:,i) = Sigma_dgiveny{i}(range_rightElbow_roty,range_rightElbow_roty);
    Sigma_tau.rightElbow_rotz(:,i) = Sigma_dgiveny{i}(range_rightElbow_rotz,range_rightElbow_rotz);
    Sigma_tau.leftElbow_roty(:,i)  = Sigma_dgiveny{i}(range_leftElbow_roty,range_leftElbow_roty);
    Sigma_tau.leftElbow_rotz(:,i)  = Sigma_dgiveny{i}(range_leftElbow_rotz,range_leftElbow_rotz);
    %
    Sigma_tau.rightWrist_rotx(:,i) = Sigma_dgiveny{i}(range_rightWrist_rotx,range_rightWrist_rotx);
    Sigma_tau.rightWrist_rotz(:,i) = Sigma_dgiveny{i}(range_rightWrist_rotz,range_rightWrist_rotz);
    Sigma_tau.leftWrist_rotx(:,i)  = Sigma_dgiveny{i}(range_leftWrist_rotx,range_leftWrist_rotx);
    Sigma_tau.leftWrist_rotz(:,i)  = Sigma_dgiveny{i}(range_leftWrist_rotz,range_leftWrist_rotz);
end

%% Save as full (no sparse) Sigma_tau 
Sigma_tau.rightAnkle_rotx = full(Sigma_tau.rightAnkle_rotx(:,range_cut_plot));
Sigma_tau.rightAnkle_roty = full(Sigma_tau.rightAnkle_roty(:,range_cut_plot));
Sigma_tau.rightAnkle_rotz = full(Sigma_tau.rightAnkle_rotz(:,range_cut_plot));
Sigma_tau.leftAnkle_rotx = full(Sigma_tau.leftAnkle_rotx(:,range_cut_plot));
Sigma_tau.leftAnkle_roty = full(Sigma_tau.leftAnkle_roty(:,range_cut_plot));
Sigma_tau.leftAnkle_rotz = full(Sigma_tau.leftAnkle_rotz(:,range_cut_plot));
%
Sigma_tau.rightKnee_roty = full(Sigma_tau.rightKnee_roty(:,range_cut_plot));
Sigma_tau.rightKnee_rotz = full(Sigma_tau.rightKnee_rotz(:,range_cut_plot));
Sigma_tau.leftKnee_roty = full(Sigma_tau.leftKnee_roty(:,range_cut_plot));
Sigma_tau.leftKnee_rotz = full(Sigma_tau.leftKnee_rotz(:,range_cut_plot));
%
Sigma_tau.rightHip_rotx = full(Sigma_tau.rightHip_rotx(:,range_cut_plot));
Sigma_tau.rightHip_roty = full(Sigma_tau.rightHip_roty (:,range_cut_plot));
Sigma_tau.rightHip_rotz = full(Sigma_tau.rightHip_rotz(:,range_cut_plot));
Sigma_tau.leftHip_rotx = full(Sigma_tau.leftHip_rotx(:,range_cut_plot));
Sigma_tau.leftHip_roty = full(Sigma_tau.leftHip_roty (:,range_cut_plot));
Sigma_tau.leftHip_rotz = full(Sigma_tau.leftHip_rotz(:,range_cut_plot));
%
Sigma_tau.L5S1_rotx = full(Sigma_tau.L5S1_rotx(:,range_cut_plot));
Sigma_tau.L5S1_roty = full(Sigma_tau.L5S1_roty (:,range_cut_plot));
%
Sigma_tau.L4L3_rotx = full(Sigma_tau.L4L3_rotx(:,range_cut_plot));
Sigma_tau.L4L3_roty = full(Sigma_tau.L4L3_roty (:,range_cut_plot));
%
Sigma_tau.L1T12_rotx = full(Sigma_tau.L1T12_rotx(:,range_cut_plot));
Sigma_tau.L1T12_roty = full(Sigma_tau.L1T12_roty (:,range_cut_plot));
%
Sigma_tau.T1C7_rotx = full(Sigma_tau.T1C7_rotx(:,range_cut_plot));
Sigma_tau.T1C7_roty = full(Sigma_tau.T1C7_roty (:,range_cut_plot));
Sigma_tau.T1C7_rotz = full(Sigma_tau.T1C7_rotz(:,range_cut_plot));
%
Sigma_tau.C1Head_rotx = full(Sigma_tau.C1Head_rotx(:,range_cut_plot));
Sigma_tau.C1Head_roty = full(Sigma_tau.C1Head_roty (:,range_cut_plot));
%
Sigma_tau.rightC7Shoulder_rotx = full(Sigma_tau.rightC7Shoulder_rotx(:,range_cut_plot));
Sigma_tau.leftC7Shoulder_rotx = full(Sigma_tau.leftC7Shoulder_rotx(:,range_cut_plot));
%
Sigma_tau.rightShoulder_rotx = full(Sigma_tau.rightShoulder_rotx(:,range_cut_plot));
Sigma_tau.rightShoulder_roty = full(Sigma_tau.rightShoulder_roty (:,range_cut_plot));
Sigma_tau.rightShoulder_rotz = full(Sigma_tau.rightShoulder_rotz(:,range_cut_plot));
Sigma_tau.leftShoulder_rotx = full(Sigma_tau.leftShoulder_rotx(:,range_cut_plot));
Sigma_tau.leftShoulder_roty = full(Sigma_tau.leftShoulder_roty (:,range_cut_plot));
Sigma_tau.leftShoulder_rotz = full(Sigma_tau.leftShoulder_rotz(:,range_cut_plot));
%
Sigma_tau.rightElbow_roty = full(Sigma_tau.rightElbow_roty(:,range_cut_plot));
Sigma_tau.rightElbow_rotz = full(Sigma_tau.rightElbow_rotz(:,range_cut_plot));
Sigma_tau.leftElbow_roty = full(Sigma_tau.leftElbow_roty(:,range_cut_plot));
Sigma_tau.leftElbow_rotz = full(Sigma_tau.leftElbow_rotz(:,range_cut_plot));
%
Sigma_tau.rightWrist_rotx = full(Sigma_tau.rightWrist_rotx(:,range_cut_plot));
Sigma_tau.rightWrist_rotz = full(Sigma_tau.rightWrist_rotz(:,range_cut_plot));
Sigma_tau.leftWrist_rotx = full(Sigma_tau.leftWrist_rotx(:,range_cut_plot));
Sigma_tau.leftWrist_rotz = full(Sigma_tau.leftWrist_rotz(:,range_cut_plot));
