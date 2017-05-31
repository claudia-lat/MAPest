function [y_simulated, Y_piece, b_Y_piece] = sim_piece_y_test(berdy, human_state, mu_dgiveny, name_sensor, name_variable)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


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

range_meas = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, name_sensor );
range_var = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, name_variable);
% range_meas = rangeOfSensorMeasurement(berdy, iDynTree.ACCELEROMETER_SENSOR, name_sensor );
% range_var = rangeOfDynamicVariable(berdy, iDynTree.LINK_BODY_PROPER_ACCELERATION, name_variable);

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
    
%     % for acceleration
%     Y_piece = full(Y(range_meas:range_meas+2,range_var:range_var+5));
%     b_Y_piece = b_Y(range_meas:range_meas+2);
    % for net ext forces
    Y_piece = full(Y(range_meas:range_meas+5,range_var:range_var+5));
    b_Y_piece = b_Y(range_meas:range_meas+5);
                                 
    y_simulated(:,i) = Y * mu_dgiveny(:,i) + b_Y;

end

end

