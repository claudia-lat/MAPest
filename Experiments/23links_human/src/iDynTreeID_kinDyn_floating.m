function [tau] = iDynTreeID_kinDyn_floating(kinDynComputation, baseOrientation, basePosition, baseVel, baseAcc, jointsQty, fext)
%IDYNTREEID_FLOATING computes the iDynTree Inverse Dynamics (ID)as implemented in
% iDynTree C++ method inverseDynamics() of kinDynComputations.
%
% Input:
% - kynDynComputation : kinDynComputation object initialized by berdy
% - baseOrientation   : orientation of the base
% - basePosition      : position of the base w.r.t. the inertial frame
% - baseVel           : 6D velocity of the base (linear vel + angular vel)
%                       expressed w.r.t. the inertial frame
% - baseAcc           : 6D acceleration of the base (linear acc + angular acc)
%                       expressed w.r.t. the inertial frame, without gravity
% - jointsQty         : q,dq (for the kinDynComputation.setRobotState) + ddq
% - fext              : 6D wrench in the body frame origin
%                       expressed w.r.t. the inertial frame
%
% Output:
% - tau               : torque


% Initialize state
q = iDynTree.VectorDynSize();
q.resize(size(jointsQty.q,1));
dq = iDynTree.VectorDynSize();
dq.resize(size(jointsQty.dq,1));

% Initialize base acceleration
baseAcc_iDynTree = iDynTree.Vector6();

% % Initialize base velocity
baseVel_iDynTree = iDynTree.Twist();

% Initialize state
ddq = iDynTree.VectorDynSize();
ddq.resize(size(jointsQty.ddq,1));

% Initialize external wrenches at the feet.
% Where not specified, by default wrench vector = zero
fext_iDynTree = iDynTree.LinkWrenches(kinDynComputation.model);
lFootIndex = kinDynComputation.model.getLinkIndex('LeftFoot');
rFootIndex = kinDynComputation.model.getLinkIndex('RightFoot');

% Define torque outputs
tau_iDynTree = iDynTree.FreeFloatingGeneralizedTorques(kinDynComputation.model); %the output
tau_iDynTree_joint = iDynTree.JointDOFsDoubleArray(kinDynComputation.model);
tau_iDynTree_joint.zero();
tau = zeros(size(jointsQty.q));

% Define parameters for setRobotState
G_T_baseRot = iDynTree.Rotation();
G_T_basePos = iDynTree.Position();
gravity = iDynTree.Vector3();
gravity.fromMatlab([0; 0; -9.81]);

samples = size(jointsQty.q ,2);

for i = 1 : samples
    q.fromMatlab(jointsQty.q(:,i));
    dq.fromMatlab(jointsQty.dq(:,i));
    % rot
    G_R_base = quat2Mat(baseOrientation(:,i));
    G_T_baseRot.fromMatlab(G_R_base);
    % pos
    G_T_basePos.fromMatlab(basePosition(:,i));
    % transf
    G_T_base = iDynTree.Transform(G_T_baseRot,G_T_basePos);
%     G_H_base = G_T_base.asHomogeneousTransform();
    
    baseVel_iDynTree.fromMatlab(baseVel(:,i));
    
    kinDynComputation.setRobotState(G_T_base,q,baseVel_iDynTree,dq,gravity);
    
    baseAcc_iDynTree.fromMatlab(baseAcc(:,i));
    
    ddq.fromMatlab(jointsQty.ddq(:,i));
    
    lFootWrench = fext_iDynTree(lFootIndex);
    lFootWrench.fromMatlab(fext.leftShoe_wrtG(:,i));
    rFootWrench = fext_iDynTree(rFootIndex);
    rFootWrench.fromMatlab(fext.rightShoe_wrtG(:,i));
    
    kinDynComputation.inverseDynamics(baseAcc_iDynTree,ddq,fext_iDynTree,tau_iDynTree);
    
    % Consider only the joint part of the computation
    tau_iDynTree_joint = tau_iDynTree.jointTorques();
    tau(:,i) = tau_iDynTree_joint.toMatlab;
end
end
