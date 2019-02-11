function [ baseLinVelocity, baseAngVelocity, baseKinDynModel ] = computeBaseVelocity( kynDynComputation, currentBerdyBase, state, G_T_base ,endEffectorFrame)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%% --- BASE VELOCITY VIA DIFFERENTIAL KINEMATIC EQUATION
% The velocity 6D of the base w.r.t. the inertail frame I (in the suit
% case i.e., G) is:
%              G_v_B = - inv(J(q)_B)* J(q)_S*\dot(s)
% where G_v_B = [G_\dot(x)_B; G_w_B]. G_w_B is the value we are looking for.

iDynTreeJacobian = iDynTree.FrameFreeFloatingJacobian(kynDynComputation.model);
iDynTreeJacobian.zero();

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
    % Compute the Jacobian J = [J(q)_B J(q)_S] from kin
    kynDynComputation.setRobotState(G_T_base.G_T_b{i,1},q,base_vel,dq,gravity);
    kynDynComputation.getFrameFreeFloatingJacobian(endEffectorFrame, iDynTreeJacobian);
    fullJacobian = iDynTreeJacobian.toMatlab();
    % Compute I_v_B
    G_v_B = - inv(fullJacobian(:,1:6))* fullJacobian(:,7:end)*state.dq(:,i);
    baseLinVelocity(:,i) = G_v_B(1:3,:);
    baseAngVelocity(:,i) = G_v_B(4:6,:);
end
end
