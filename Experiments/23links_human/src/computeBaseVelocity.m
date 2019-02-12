function [ baseLinVelocity, baseAngVelocity, baseKinDynModel ] = computeBaseVelocity( kynDynComputation, currentBerdyBase, state, G_T_base ,constraints)
%COMPUTEBASEVELOCITY computes the 6D base velocity via differential
%kinematic equation by considering 2 constraints.

% The velocity of the base w.r.t. the global frame G is
% G_v_B = [G_\dot(x)_B; G_w_B].

endEffectorFrameLF = constraints.endEffectorFrameLF;
kynDynComputationLF = kynDynComputation;
iDynTreeJacobianLF = iDynTree.FrameFreeFloatingJacobian(kynDynComputationLF.model);
iDynTreeJacobianLF.zero();

endEffectorFrameRF = constraints.endEffectorFrameRF;
kynDynComputationRF = kynDynComputation;
iDynTreeJacobianRF = iDynTree.FrameFreeFloatingJacobian(kynDynComputationRF.model);
iDynTreeJacobianRF.zero()

q  = iDynTree.JointPosDoubleArray(kynDynComputation.model);
dq = iDynTree.JointDOFsDoubleArray(kynDynComputation.model);
base_vel = iDynTree.Twist();
gravity = iDynTree.Vector3();
gravity.fromMatlab([0; 0; -9.81]);

samples = size(state.q ,2);
baseLinVelocity = zeros(3,samples);
baseAngVelocity = zeros(3,samples);

kynDynComputation.setFloatingBase(currentBerdyBase);
baseKinDynModel = kynDynComputation.getFloatingBase();
% Consistency check
% berdy.model base and kynDynComputation.model have to be consistent!
if currentBerdyBase ~= baseKinDynModel
    error(strcat('[ERROR] The berdy model base (',currentBerdyBase,') and the kinDyn model base (',baseKinDynModel,') do not match!'));
end

for i = 1 : samples
    q.fromMatlab(state.q(:,i));
    dq.fromMatlab(state.dq(:,i));
    
    base_vel.fromMatlab(zeros(6,1));
    
    % Compute the Jacobian J = [J(q)_B J(q)_S] from kin per each constraint
    kynDynComputationLF.setRobotState(G_T_base.G_T_b{i,1},q,base_vel,dq,gravity);
    kynDynComputationLF.getFrameFreeFloatingJacobian(endEffectorFrameLF, iDynTreeJacobianLF);
    fullJacobianLF = iDynTreeJacobianLF.toMatlab();
    
    kynDynComputationRF.setRobotState(G_T_base.G_T_b{i,1},q,base_vel,dq,gravity);
    kynDynComputationRF.getFrameFreeFloatingJacobian(endEffectorFrameRF, iDynTreeJacobianRF);
    fullJacobianRF = iDynTreeJacobianRF.toMatlab();
    
    baseJacobianCombined   = [fullJacobianLF(:,1:6); fullJacobianRF(:,1:6)];
    jointsJacobianCombined = [fullJacobianLF(:,7:end);fullJacobianRF(:,7:end)];
    
    % Compute G_v_B
    G_v_B = - pinv(baseJacobianCombined,1e-4) * jointsJacobianCombined * state.dq(:,i);
    baseLinVelocity(:,i) = G_v_B(1:3,:);
    baseAngVelocity(:,i) = G_v_B(4:6,:);
end
end
