
% Script to extract via berdy the estimated torque by MAP.
mu_dgiveny_berdy = iDynTree.VectorDynSize();
q_berdy = iDynTree.VectorDynSize();
tau_berdy = iDynTree.VectorDynSize(); %the output

% Resize vector
mu_dgiveny_berdy.resize(berdy.getNrOfDynamicVariables());
q_berdy.resize(size(synchroData(1).q,1));
tau_berdy.resize(size(synchroData(1).q,1));

% estimatedTauFromBerdy = struct;
for blockIdx = 1 : block.nrOfBlocks
    nrOfSamples  = size(data(blockIdx).y, 2);
    tau_vector = zeros(size(synchroData(blockIdx).q));
    tauFromBerdy = zeros(size(synchroData(blockIdx).q));
    for i = 1 : nrOfSamples
        mu_dgiveny_berdy.fromMatlab(estimation(blockIdx).mu_dgiveny(:,i));
        q_berdy.fromMatlab(synchroData(blockIdx).q(:,i));
        tau_berdy.fromMatlab(tau_vector(:,i));
        berdy.extractJointTorquesFromDynamicVariables(mu_dgiveny_berdy,q_berdy, tau_berdy);
        tauFromBerdy(:,i) = tau_berdy.toMatlab();
    end
    estimatedVariables.tau(blockIdx).label  = selectedJoints;
    estimatedVariables.tau(blockIdx).values = tauFromBerdy;
end
