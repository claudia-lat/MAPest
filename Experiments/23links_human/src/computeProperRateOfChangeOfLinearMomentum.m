function properDotL_lin = computeProperRateOfChangeOfLinearMomentum(kinDynComputation, humanModel, state, baseVelocity, G_T_b)
%COMPUTEPROPERRATEOFCHANGEOFLINEARMOMENTUM computes the proper rate of
%chenge of the linear part of the momentum, such as
%
%          properDotL_lin = m * b_R_G * (G_linAccCOM - G_g)
%
% where:
% - G_linAccCOM is the proper linear CoM acceleration w.r.t. the global
%               suit frame G;
% - b_R_G is the rotation matrix to transform a 3x1 vector from the global
%         frame to the base frame;

q  = iDynTree.JointPosDoubleArray(kinDynComputation.model);
dq = iDynTree.JointDOFsDoubleArray(kinDynComputation.model);
base_vel = iDynTree.Twist();
gravity = iDynTree.Vector3();
gravityMatlab = [0; 0; -9.81];
gravity.fromMatlab(gravityMatlab);

G_R_b = iDynTree.Rotation();
biasAccCOM = iDynTree.Vector3();
samples = size(state.q ,2);
properDotL_lin = zeros(3,samples);

for i = 1 : samples
    q.fromMatlab(state.q(:,i));
    dq.fromMatlab(state.dq(:,i));
    base_vel.fromMatlab(baseVelocity(:,i));
    kinDynComputation.setRobotState(G_T_b{i,1},q,base_vel,dq,gravity);
    % Get rotation matrix
    G_R_b = G_T_b{i,1}.getRotation;
    % Get center of mass bias acceleration
    biasAccCOM = kinDynComputation.getCenterOfMassBiasAcc();
    % Compute dL_lin
    properDotL_lin(:,i) =  humanModel.getTotalMass * G_R_b.toMatlab' * (biasAccCOM.toMatlab - gravityMatlab);
end
