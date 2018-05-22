
% Script to extract via berdy the estimated torque by MAP.

bucket.nrOfSamples = size(data(blockIdx).y, 2);
mu_dgiveny_berdy = iDynTree.VectorDynSize();
q_berdy = iDynTree.VectorDynSize();
tau_berdy = iDynTree.VectorDynSize(); %the output
tau_vector = zeros(size(synchroData(blockIdx).q));
tauFromBerdy = zeros(size(synchroData(blockIdx).q));

for i = 1 : bucket.nrOfSamples
    mu_dgiveny_berdy.fromMatlab(estimation(blockIdx).mu_dgiveny(:,i));
    q_berdy.fromMatlab(synchroData(blockIdx).q(:,i));
    tau_berdy.fromMatlab(tau_vector(:,i));

    berdy.extractJointTorquesFromDynamicVariables(mu_dgiveny_berdy,q_berdy, tau_berdy);
    tauFromBerdy(:,i) = tau_berdy.toMatlab;
end

% load struct
computedTauFromBerdy = struct;
for i = 1 : size(tauFromBerdy,1)
    computedTauFromBerdy(i).label = selectedJoints(i);
    computedTauFromBerdy(i).values = tauFromBerdy(i,:);
end
