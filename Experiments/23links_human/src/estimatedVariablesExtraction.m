function [estimated_variables] = estimatedVariablesExtraction(berdy, currentBase, mu_dgiveny)
%ESTIMATEDVARIABLESEXTRACTION creates a structure where is specified the
% type/value of the extracted variable from the vector mu_dgiveny (i.e., d).
% This function is taylored for 48-DoF model (it has to be modified for
% the 66-DoF case!) and for the fixed base formalism.

% The vector d is ordered in the following way
%                 d  = [d_1, d_2, ..., d_n], i = 1,...,n number of links
% with:
%                d_i = [a_i, fB_i, f_i, tau_i, fx_i, ddq_i]
% where:
%         a_i   is the iDynTree.LINK_BODY_PROPER_ACCELERATION
%         fB_i  is iDynTree.NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV
%         f_i   is iDynTree.JOINT_WRENCH
%         tau_i is iDynTree.DOF_TORQUE
%         fx_i  is iDynTree.NET_EXT_WRENCH
%         ddq_i is iDynTree.DOF_ACCELERATION

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
         
% Since we are in fixed base formalism, the quantities in the vector d 
% related to the links do not exist in the fixed base! We need to remove 
% them from the link list.
list_link(strncmpi(list_link,currentBase,20)) = [];

%% Extraction of LINK_BODY_PROPER_ACCELERATION
% 6D acceleration of the i-th link
estimated_variables.acc = cell(length(list_link),1);
range = zeros(length(list_link),1);
for i = 1 : length(list_link)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.LINK_BODY_PROPER_ACCELERATION, list_link{i}));
    estimated_variables.acc{i}.label = list_link{i};
    estimated_variables.acc{i}.acc   = mu_dgiveny(range(i):range(i)+5, :); 
end

%% Extraction of NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV
% 6D net force acting on the i-th link
estimated_variables.netForce = cell(length(list_link),1);
range = zeros(length(list_link),1);
for i = 1 : length(list_link)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV, list_link{i}));
    estimated_variables.netForce{i}.label = list_link{i};
    estimated_variables.netForce{i}.netForce   = mu_dgiveny(range(i):range(i)+5, :); 
end

%% Extraction of JOINT_WRENCH
% 6D force exchanged between (i-1)-th and i-th links through the i-th joint
estimated_variables.intForce.rotx = cell(length(list_rotx),1);
range = zeros(length(list_rotx),1);
for i = 1 : length(list_rotx)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.JOINT_WRENCH, list_rotx{i}));
    estimated_variables.intForce.rotx{i}.label = list_rotx{i};
    estimated_variables.intForce.rotx{i}.intForce   = mu_dgiveny(range(i):range(i)+5, :); 
end

estimated_variables.intForce.roty = cell(length(list_roty),1);
range = zeros(length(list_roty),1);
for i = 1 : length(list_roty)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.JOINT_WRENCH, list_roty{i}));
    estimated_variables.intForce.roty{i}.label = list_roty{i};
    estimated_variables.intForce.roty{i}.intForce   = mu_dgiveny(range(i):range(i)+5, :); 
end

estimated_variables.intForce.rotz = cell(length(list_rotz),1);
range = zeros(length(list_rotz),1);
for i = 1 : length(list_rotz)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.JOINT_WRENCH, list_rotz{i}));
    estimated_variables.intForce.rotz{i}.label = list_rotz{i};
    estimated_variables.intForce.rotz{i}.intForce   = mu_dgiveny(range(i):range(i)+5, :); 
end

%% Extraction of DOF_TORQUE
% Torque of the i-th joint
estimated_variables.tau.rotx = cell(length(list_rotx),1);
range = zeros(length(list_rotx),1);
for i = 1 : length(list_rotx)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, list_rotx{i}));
    estimated_variables.tau.rotx{i}.label = list_rotx{i};
    estimated_variables.tau.rotx{i}.tau   = mu_dgiveny(range(i), :); 
end

estimated_variables.tau.roty = cell(length(list_roty),1);
range = zeros(length(list_roty),1);
for i = 1 : length(list_roty)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, list_roty{i}));
    estimated_variables.tau.roty{i}.label = list_roty{i};
    estimated_variables.tau.roty{i}.tau   = mu_dgiveny(range(i), :); 
end

estimated_variables.tau.rotz = cell(length(list_rotz),1);
range = zeros(length(list_rotz),1);
for i = 1 : length(list_rotz)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, list_rotz{i}));
    estimated_variables.tau.rotz{i}.label = list_rotz{i};
    estimated_variables.tau.rotz{i}.tau   = mu_dgiveny(range(i), :); 
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

%% Extraction of DOF_ACCELERATION
% Joint acceleration acting on the i-th joint
estimated_variables.ddq.rotx = cell(length(list_rotx),1);
range = zeros(length(list_rotx),1);
for i = 1 : length(list_rotx)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, list_rotx{i}));
    estimated_variables.ddq.rotx{i}.label = list_rotx{i};
    estimated_variables.ddq.rotx{i}.ddq   = mu_dgiveny(range(i), :); 
end

estimated_variables.ddq.roty = cell(length(list_roty),1);
range = zeros(length(list_roty),1);
for i = 1 : length(list_roty)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, list_roty{i}));
    estimated_variables.ddq.roty{i}.label = list_roty{i};
    estimated_variables.ddq.roty{i}.ddq   = mu_dgiveny(range(i), :); 
end

estimated_variables.ddq.rotz = cell(length(list_rotz),1);
range = zeros(length(list_rotz),1);
for i = 1 : length(list_rotz)
    range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, list_rotz{i}));
    estimated_variables.ddq.rotz{i}.label = list_rotz{i};
    estimated_variables.ddq.rotz{i}.ddq   = mu_dgiveny(range(i), :); 
end

end
