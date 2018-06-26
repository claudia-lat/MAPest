function [ baseAngVelocity, baseKinDynModel ] = computeBaseAngularVelocity( kynDynComputation, currentBerdyBase, state, endEffectorFrame)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%% --- BASE ANGULAR VELOCITY VIA DIFFERENTIAL KINEMATIC EQUATION
% The velocity 6D of the base w.r.t. the inertail frame I is:
%              I_v_B = - inv(J(q)_B)* J(q)_S*\dot(s)
% where I_v_B = [I_\dot(x)_B; I_w_B]. I_w_B is the value we are looking for.

iDynTreeJacobian = iDynTree.FrameFreeFloatingJacobian(kynDynComputation.model);
iDynTreeJacobian.zero();

q  = iDynTree.JointPosDoubleArray(kynDynComputation.model);
dq = iDynTree.JointDOFsDoubleArray(kynDynComputation.model);
gravityZero = iDynTree.Vector3();
gravityZero.zero();

samples = size(state.q ,2);
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
    % Compute the Jacobian J = [J(q)_B J(q)_S] from kin
    kynDynComputation.setRobotState(q,dq,gravityZero);
    kynDynComputation.getFrameFreeFloatingJacobian(endEffectorFrame, iDynTreeJacobian);
    fullJacobian = iDynTreeJacobian.toMatlab();
    % Compute I_w_B
    I_v_B = - inv(fullJacobian(:,1:6))* fullJacobian(:,7:end)*state.dq(:,i);
    baseAngVelocity(:,i) = I_v_B(4:6,:);
end
end
