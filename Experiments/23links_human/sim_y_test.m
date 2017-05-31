function [ y_simulated ] = sim_y_test( berdy, human_state, mu_dgiveny )


% Set gravity 
gravity = [0 0 -9.81];
grav  = iDynTree.Vector3();
grav.fromMatlab(gravity);

% Set matrices
berdyMatrices       = struct;
berdyMatrices.D     = iDynTree.MatrixDynSize();
berdyMatrices.b_D   = iDynTree.VectorDynSize();
berdyMatrices.Y     = iDynTree.MatrixDynSize();
berdyMatrices.b_Y   = iDynTree.VectorDynSize();

berdy.resizeAndZeroBerdyMatrices(berdyMatrices.D,...
                                 berdyMatrices.b_D,...
                                 berdyMatrices.Y,...
                                 berdyMatrices.b_Y);
                             
                             
q  = iDynTree.JointPosDoubleArray(berdy.model());
dq = iDynTree.JointDOFsDoubleArray(berdy.model());


for i = 1: length(human_state.q)

    q.fromMatlab(human_state.q(:,i));
    dq.fromMatlab(human_state.dq(:,i));
    
    berdy.updateKinematicsFromTraversalFixedBase(q,dq,grav);
    
    berdy.getBerdyMatrices(berdyMatrices.D,...
                           berdyMatrices.b_D,...
                           berdyMatrices.Y,...
                           berdyMatrices.b_Y);   
                       
    Y_nonsparse = berdyMatrices.Y.toMatlab();

    Y   = sparse(Y_nonsparse);
    b_Y = berdyMatrices.b_Y.toMatlab();
                                        
    y_simulated(:,i) = Y * mu_dgiveny(:,i) + b_Y;

end


end

