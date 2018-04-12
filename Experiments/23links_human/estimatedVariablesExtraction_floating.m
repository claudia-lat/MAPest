function [estimated_variables] = estimatedVariablesExtraction_floating(berdy, mu_dgiveny)
%ESTIMATEDVARIABLESEXTRACTION_FLOATING creates a structure where is specified the
% type/value of the extracted variable from the vector mu_dgiveny (i.e., d).
% This function is taylored for 48-DoF model (it has to be modified for
% the 66-DoF case!) and for the fixed base formalism.

% The vector d is ordered in the following way
%                 d  = [d_1, d_2, ..., d_n], i = 1,...,n number of links
% with:
%                d_i = [a_i, f_i, fx_i]
% where:
%         a_i   is the iDynTree.LINK_BODY_PROPER_CLASSICAL_ACCELERATION
%         f_i   is iDynTree.JOINT_WRENCH
%         fx_i  is iDynTree.NET_EXT_WRENCH

%% List of joint names
% Joints are randomly ordered!
list_rotx = {'jRightAnkle_rotx';
             'jLeftAnkle_rotx';
             'jRightHip_rotx';
             'jLeftHip_rotx';
             'jL5S1_rotx';
             'jL4L3_rotx';
             'jL1T12_rotx';
             'jT9T8_rotx';
             'jT1C7_rotx';
             'jC1Head_rotx';
             'jRightC7Shoulder_rotx';
             'jLeftC7Shoulder_rotx';
             'jRightShoulder_rotx';
             'jLeftShoulder_rotx';
             'jRightWrist_rotx';
             'jLeftWrist_rotx'
             };

list_roty = {'jRightAnkle_roty';
             'jLeftAnkle_roty';
             'jRightKnee_roty';
             'jLeftKnee_roty';
             'jRightHip_roty';
             'jLeftHip_roty';
             'jL5S1_roty';
             'jL4L3_roty';
             'jL1T12_roty';
             'jT9T8_roty';
             'jT1C7_roty';
             'jC1Head_roty';
             'jRightShoulder_roty';
             'jLeftShoulder_roty';
             'jRightElbow_roty';
             'jLeftElbow_roty'
             };

list_rotz = {'jRightAnkle_rotz';
             'jLeftAnkle_rotz';
             'jRightKnee_rotz';
             'jLeftKnee_rotz';
             'jRightHip_rotz';
             'jLeftHip_rotz';
             'jT9T8_rotz';
             'jT1C7_rotz';
             'jRightShoulder_rotz';
             'jLeftShoulder_rotz';
             'jRightElbow_rotz';
             'jLeftElbow_rotz';
             'jRightWrist_rotz';
             'jLeftWrist_rotz'
             };
        
%% List of links
% Links are randomly ordered!
list_link = {'LeftFoot';
             'LeftFoot_f2';
             'LeftToe';
             'LeftFoot_f1';
             'LeftLowerLeg';
             'LeftLowerLeg_f1';
             'LeftUpperLeg';
             'LeftUpperLeg_f2';
             'LeftUpperLeg_f1';
             'Pelvis';
             'L5_f1';
             'RightUpperLeg_f1';
             'RightUpperLeg_f2';
             'RightUpperLeg';
             'RightLowerLeg_f1';
             'RightLowerLeg';
             'RightFoot_f1';
             'RightFoot_f2';
             'RightFoot';
             'RightToe';
             'L5';
             'L3_f1';
             'L3';
             'T12_f1';
             'T12';
             'T8_f1';
             'T8_f2';
             'T8';
             'Neck_f1';
             'RightShoulder';
             'LeftShoulder';
             'LeftUpperArm_f1';
             'LeftUpperArm_f2';
             'LeftUpperArm';
             'LeftForeArm_f1';
             'LeftForeArm';
             'LeftHand_f1';
             'LeftHand';
             'RightUpperArm_f1';
             'RightUpperArm_f2';
             'RightUpperArm';
             'RightForeArm_f1';
             'RightForeArm';
             'RightHand_f1';
             'RightHand';
             'Neck_f2';
             'Neck';
             'Head_f1';
             'Head'
             };

%% Extraction of LINK_BODY_PROPER_CLASSICAL_ACCELERATION
% 6D acceleration of the i-th link
estimated_variables.acc = cell(length(list_link),1);
range = zeros(length(list_link),1);
for i = 1 : length(list_link)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.LINK_BODY_PROPER_CLASSICAL_ACCELERATION, list_link{i}));
    estimated_variables.acc{i}.label = list_link{i};
    estimated_variables.acc{i}.acc   = mu_dgiveny(range(i):range(i)+5, :); 
end

%% Extraction of JOINT_WRENCH
% 6D force exchanged between (i-1)-th and i-th links through the i-th
% joint.
% This section computes simultaneously the torque estimation given the 
% estimation of the internal forces.

% Since we are working in floating base formalism, we do not have the 
% estimation of the torque tau_i in the mu_dgiveny vector but we can obtain 
% it as a projection of the f_i (that is present in the estimation vector),
% such that:
%                     tau_i = (S_i)^T * f_i
%
% where S_i is the motion freedom subspace:
% - S_i_x = [0 0 0 1 0 0] if the allowed motion is a rotation around x,
% - S_i_y = [0 0 0 0 1 0] if the allowed motion is a rotation around y,
% - S_i_z = [0 0 0 0 0 1] if the allowed motion is a rotation around z.

estimated_variables.intForce.rotx = cell(length(list_rotx),1);
range = zeros(length(list_rotx),1);
for i = 1 : length(list_rotx)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.JOINT_WRENCH, list_rotx{i}));
    estimated_variables.intForce.rotx{i}.label = list_rotx{i};
    estimated_variables.intForce.rotx{i}.intForce   = mu_dgiveny(range(i):range(i)+5, :);
    estimated_variables.intForce.rotx{i}.projected_tau   = mu_dgiveny(range(i)+3, :); 
    % chosing the 4th element is equal to multiply by [0 0 0 1 0 0].
end

estimated_variables.intForce.roty = cell(length(list_roty),1);
range = zeros(length(list_roty),1);
for i = 1 : length(list_roty)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.JOINT_WRENCH, list_roty{i}));
    estimated_variables.intForce.roty{i}.label = list_roty{i};
    estimated_variables.intForce.roty{i}.intForce   = mu_dgiveny(range(i):range(i)+5, :);
    estimated_variables.intForce.roty{i}.projected_tau   = mu_dgiveny(range(i)+4, :); 
    % chosing the 5th element is equal to multiply by [0 0 0 0 1 0].
end

estimated_variables.intForce.rotz = cell(length(list_rotz),1);
range = zeros(length(list_rotz),1);
for i = 1 : length(list_rotz)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.JOINT_WRENCH, list_rotz{i}));
    estimated_variables.intForce.rotz{i}.label = list_rotz{i};
    estimated_variables.intForce.rotz{i}.intForce   = mu_dgiveny(range(i):range(i)+5, :);
    estimated_variables.intForce.rotz{i}.projected_tau   = mu_dgiveny(range(i)+5, :); 
    % chosing the 6th element is equal to multiply by [0 0 0 0 0 1].
end

%% Extraction of NET_EXT_WRENCH
% 6D external force acting on the i-th link
estimated_variables.extForce = cell(length(list_link),1);
range = zeros(length(list_link),1);
for i = 1 : length(list_link)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, list_link{i}));
    estimated_variables.extForce{i}.label = list_link{i};
    estimated_variables.extForce{i}.extForce   = mu_dgiveny(range(i):range(i)+5, :); 
end

end
