function [ mu_dgiveny, Sigma_dgiveny] = MAPcomputation(berdy,state,y,priors)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

% set gravity 
gravM = [0 0 -9.81];
grav = iDynTree.Vector3();
grav.fromMatlab(gravM);

%set matrices
berdyMatrices       = struct;
berdyMatrices.D     = iDynTree.MatrixDynSize();
berdyMatrices.b_D   = iDynTree.VectorDynSize();
berdyMatrices.Y     = iDynTree.MatrixDynSize();
berdyMatrices.b_Y   = iDynTree.VectorDynSize();

berdy.resizeAndZeroBerdyMatrices(berdyMatrices.D,...
                                 berdyMatrices.b_D,...
                                 berdyMatrices.Y,...
                                 berdyMatrices.b_Y);
  
% title
mud = priors.mud;
Sigmad_inv = inv(priors.Sigmad);
SigmaD_inv = inv(priors.SigmaD);
Sigmay_inv = inv(priors.Sigmay);

samples = size(y, 2); 

nrOfDynVariables = berdy.getNrOfDynamicVariables();

% allocate outputs
mu_dgiveny = zeros(nrOfDynVariables, samples);
Sigma_dgiveny = zeros(nrOfDynVariables, nrOfDynVariables, samples);


q  = iDynTree.JointPosDoubleArray(berdy.model());
dq = iDynTree.JointDOFsDoubleArray(berdy.model());


for i = 1 : samples
    
    q.fromMatlab(state.q(:,i));
    dq.fromMatlab(state.dq(:,i));
    
    berdy.updateKinematicsFromTraversalFixedBase(q,dq,grav);

    berdy.getBerdyMatrices(berdyMatrices.D,...
                   berdyMatrices.b_D,...
                   berdyMatrices.Y,...
                   berdyMatrices.b_Y);

    D = berdyMatrices.D.toMatlab();
    b_D = berdyMatrices.b_D.toMatlab();
    Y = berdyMatrices.Y.toMatlab();
    b_Y = berdyMatrices.b_Y.toMatlab();


    SigmaBarD_inv   = D' * SigmaD_inv * D + Sigmad_inv;
    muBarD          = SigmaBarD_inv \ (Sigmad_inv * mud - D' * SigmaD_inv * b_D);

    Sigma_dgiveny(:,:,i)  = inv(SigmaBarD_inv + Y' * Sigmay_inv * Y);
    mu_dgiveny(:,i)       = Sigma_dgiveny(:,:,i) * (Y' * Sigmay_inv * (y(:,i) - b_Y) + SigmaBarD_inv * muBarD);

end

end

