function [torquesM, baseReactionForceM ] = iDynTreeID(dyntree, qM, dqM, ddqM)
%IDYNTREEID computes the inverse dynamics using the iDynTree DynamicsComputations
% class.

% set gravity
grav = iDynTree.SpatialAcc();
grav.zero();
grav.setVal(2,-9.81);

dofs = dyntree.getNrOfDegreesOfFreedom();
q = iDynTree.VectorDynSize(dofs);
dq = iDynTree.VectorDynSize(dofs);
ddq = iDynTree.VectorDynSize(dofs);

torques = iDynTree.VectorDynSize(dofs);
baseReactionForce = iDynTree.Wrench();
    
torquesM =zeros(dofs, size(qM,2));
baseReactionForceM =zeros(6, size(qM,2));

for i = 1: size(qM,2)
    q.fromMatlab(qM(:,i));
    dq.fromMatlab(dqM(:,i));
    ddq.fromMatlab(ddqM(:,i));

    dyntree.setRobotState(q,dq,ddq,grav);

    % compute id with inverse dynamics method
    dyntree.inverseDynamics(torques,baseReactionForce);

    torquesM(:,i) = torques.toMatlab();
    baseReactionForceM(:,i) = baseReactionForce.toMatlab();
end
end

