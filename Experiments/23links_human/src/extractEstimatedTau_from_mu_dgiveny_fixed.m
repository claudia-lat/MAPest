function tau = extractEstimatedTau_from_mu_dgiveny_fixed(berdy, dJointOrder, mu_dgiveny)
%EXTRACTEDESTIMATEDtau_FROM_MUDGIVENY_fixed extracts the estimated torque
% into the vector estimated by MAP.

nrOfJoints = size(dJointOrder,1);
range = zeros(nrOfJoints,1);
nrOfSamples  = size(mu_dgiveny  ,2);

tau = zeros(size(dJointOrder,1), nrOfSamples);
for i = 1 : nrOfJoints
    range = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, dJointOrder{i}));
    tau(i,:) = mu_dgiveny(range,:);
end
end
