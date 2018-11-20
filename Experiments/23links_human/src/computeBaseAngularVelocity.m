function [ baseAngVelocity, baseKinDynModel ] = computeBaseAngularVelocity( kinDynComputation, currentBerdyBase, state, endEffectorFrame)
%COMPUTEBASEANGULARVELOCITY computes the angular velocity of the model Base
%via differential kinematic equation in [rad/s].
%
% The velocity 6D of the base w.r.t. the inertial frame I is:
%
%              I_v_B = - inv(J(q)_B)* J(q)_S*\dot(s)
%
% where I_v_B = [I_\dot(x)_B; I_w_B].

% Inputs: 
%  - kinDynComputation: berdy object
%  - currentBerdyBase:  base of the model you want to compute the angular velocity
%  - state:             Matlab struct containing q and dq
%  - endEffectorFrame:  frame of an end effector whose velocity is assumed to
%                       be zero (e.g., a frame associated to a link that is in 
%                       fixed contact with the ground).
% Outputs: 
%  - baseAngVelocity:   I_w_B, angular velocity of the base B w.r.t. the
%                       inertail frame I, in [rad/s].

iDynTreeJacobian = iDynTree.FrameFreeFloatingJacobian(kinDynComputation.model);
iDynTreeJacobian.zero();

q  = iDynTree.JointPosDoubleArray(kinDynComputation.model);
dq = iDynTree.JointDOFsDoubleArray(kinDynComputation.model);
gravityZero = iDynTree.Vector3();
gravityZero.zero();

samples = size(state.q ,2);
baseAngVelocity = zeros(3,samples);

kinDynComputation.setFloatingBase(currentBerdyBase);
baseKinDynModel = kinDynComputation.getFloatingBase();
% Consistency check
% berdy.model base and kinDynComputation.model have to be consistent!
if currentBerdyBase ~= baseKinDynModel
    error(strcat('[ERROR] The berdy model base (',currentBerdyBase,') and the kinDyn model base (',baseKinDynModel,') do not match!'));
end

for i = 1 : samples
    q.fromMatlab(state.q(:,i));
    dq.fromMatlab(state.dq(:,i));
    % Compute the Jacobian J = [J(q)_B J(q)_S] from kin
    kinDynComputation.setRobotState(q,dq,gravityZero);
    kinDynComputation.getFrameFreeFloatingJacobian(endEffectorFrame, iDynTreeJacobian);
    fullJacobian = iDynTreeJacobian.toMatlab();
    % Compute I_w_B
    I_v_B = - inv(fullJacobian(:,1:6))* fullJacobian(:,7:end)*state.dq(:,i);
    baseAngVelocity(:,i) = I_v_B(4:6,:);
end
end
